import React, { createContext, useState, useContext, useRef, useCallback } from 'react';

const ROSContext = createContext();

export const useROS = () => {
  const context = useContext(ROSContext);
  if (!context) {
    throw new Error('useROS must be used within a ROSProvider');
  }
  return context;
};

export const ROSProvider = ({ children }) => {
  const [connectionInfo, setConnectionInfo] = useState({
    ip: '',
    rosbridgePort: '',
    videoPort: '',
  });
  const [subscribedTopics, setSubscribedTopics] = useState([]);
  const [isConnected, setIsConnected] = useState(false);
  const wsRef = useRef(null);

  const handleWebSocketMessage = useCallback((event) => {
    try {
      const data = JSON.parse(event.data);
      
      if (data.topic && data.msg) {
        setSubscribedTopics(prevTopics => 
          prevTopics.map(t => 
            t.topic === data.topic 
              ? { ...t, lastMsg: data.msg }
              : t
          )
        );
      }
    } catch (error) {
      console.error('Error parsing WebSocket message:', error);
    }
  }, []);

  const connectToROS = (ip, rosbridgePort, videoPort) => {
    return new Promise((resolve, reject) => {
      const wsUrl = `ws://${ip}:${rosbridgePort}`;
      const ws = new WebSocket(wsUrl);

      ws.onopen = () => {
        console.log('WebSocket connected');
        setIsConnected(true);
        setConnectionInfo({ 
          ip: ip, 
          rosbridgePort: rosbridgePort, 
          videoPort: videoPort 
        });
        wsRef.current = ws;
        ws.onmessage = handleWebSocketMessage;
        resolve(ws);
      };

      ws.onerror = (error) => {
        console.error('WebSocket error:', error);
        reject(new Error('Connection failed'));
      };

      ws.onclose = () => {
        console.log('WebSocket closed');
        setIsConnected(false);
        if (wsRef.current === ws) {
          wsRef.current = null;
        }
      };
    });
  };

  const subscribeToTopic = (topicName, topicType) => {
    if (!wsRef.current || wsRef.current.readyState !== WebSocket.OPEN) {
      console.error('WebSocket is not connected');
      return;
    }

    const alreadySubscribed = subscribedTopics.some(t => t.topic === topicName);
    if (alreadySubscribed) {
      console.log('Already subscribed to', topicName);
      return;
    }

    const subscribeMsg = {
      op: 'subscribe',
      topic: topicName,
      type: topicType,
    };

    wsRef.current.send(JSON.stringify(subscribeMsg));

    setSubscribedTopics(prev => [
      ...prev,
      {
        topic: topicName,
        type: topicType,
        lastMsg: null,
      },
    ]);

    console.log('Subscribed to', topicName);
  };

  const unsubscribeFromTopic = (topicName) => {
    if (!wsRef.current || wsRef.current.readyState !== WebSocket.OPEN) {
      return;
    }

    const unsubscribeMsg = {
      op: 'unsubscribe',
      topic: topicName,
    };

    wsRef.current.send(JSON.stringify(unsubscribeMsg));

    setSubscribedTopics(prev => prev.filter(t => t.topic !== topicName));
    console.log('Unsubscribed from', topicName);
  };

  const disconnect = () => {
    if (wsRef.current) {
      wsRef.current.close();
      wsRef.current = null;
    }
    setIsConnected(false);
    setSubscribedTopics([]);
    setConnectionInfo({ ip: '', rosbridgePort: '', videoPort: '' });
  };

  const value = {
    connectionInfo,
    subscribedTopics,
    isConnected,
    connectToROS,
    subscribeToTopic,
    unsubscribeFromTopic,
    disconnect,
    ws: wsRef.current,
  };

  return <ROSContext.Provider value={value}>{children}</ROSContext.Provider>;
};