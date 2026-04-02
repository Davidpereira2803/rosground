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
  const [availableTopics, setAvailableTopics] = useState([]);
  const [isConnected, setIsConnected] = useState(false);
  const wsRef = useRef(null);
  const topicTypeRequestRef = useRef(null);
  const connectTimeoutRef = useRef(null);

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

      if (data.op === 'service_response') {
        if (data.service === '/rosapi/topics') {
          const topics = data.values?.topics || [];
          console.log('Discovered topics:', topics);
          setAvailableTopics(topics);
        }
      }
    } catch (error) {
      console.error('Error parsing WebSocket message:', error);
    }
  }, []);

  const discoverTopics = useCallback(() => {
    if (!wsRef.current || wsRef.current.readyState !== WebSocket.OPEN) {
      console.warn('Cannot discover topics: WebSocket not connected');
      return;
    }

    const request = {
      op: 'call_service',
      service: '/rosapi/topics',
      args: {}
    };

    console.log('Requesting topic list...');
    wsRef.current.send(JSON.stringify(request));
  }, []);

  const getTopicType = useCallback((topicName, callback) => {
    if (!wsRef.current || wsRef.current.readyState !== WebSocket.OPEN) {
      console.warn('Cannot get topic type: WebSocket not connected');
      return;
    }

    if (topicTypeRequestRef.current) {
      const { timeoutId, handleResponse } = topicTypeRequestRef.current;
      clearTimeout(timeoutId);
      wsRef.current.removeEventListener('message', handleResponse);
      topicTypeRequestRef.current = null;
    }

    const request = {
      op: 'call_service',
      service: '/rosapi/topic_type',
      args: { topic: topicName }
    };

    let timeoutId;
    const handleResponse = (event) => {
      try {
        const data = JSON.parse(event.data);
        if (data.op === 'service_response' && data.service === '/rosapi/topic_type') {
          const type = data.values?.type || '';
          clearTimeout(timeoutId);
          wsRef.current.removeEventListener('message', handleResponse);
          if (topicTypeRequestRef.current?.handleResponse === handleResponse) {
            topicTypeRequestRef.current = null;
          }
          callback(type);
        }
      } catch (error) {
        console.error('Error parsing topic type response:', error);
      }
    };

    wsRef.current.addEventListener('message', handleResponse);
    wsRef.current.send(JSON.stringify(request));

    timeoutId = setTimeout(() => {
      if (wsRef.current) {
        wsRef.current.removeEventListener('message', handleResponse);
        if (topicTypeRequestRef.current?.handleResponse === handleResponse) {
          topicTypeRequestRef.current = null;
        }
        console.warn(`Topic type request timeout for ${topicName}`);
        callback('');
      }
    }, 5000);

    topicTypeRequestRef.current = { timeoutId, handleResponse };

}, []);

  const connectToROS = (ip, rosbridgePort, videoPort) => {
    return new Promise((resolve, reject) => {
      if (connectTimeoutRef.current) {
        clearTimeout(connectTimeoutRef.current);
        connectTimeoutRef.current = null;
      }

      if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
        wsRef.current.close();
        wsRef.current = null;
      }

      const wsUrl = `ws://${ip}:${rosbridgePort}`;
      const ws = new WebSocket(wsUrl);
      let settled = false;

      const finish = (handler) => {
        if (settled) {
          return;
        }
        settled = true;
        if (connectTimeoutRef.current) {
          clearTimeout(connectTimeoutRef.current);
          connectTimeoutRef.current = null;
        }
        handler();
      };

      ws.onopen = () => {
        finish(() => {
          wsRef.current = ws;
          setIsConnected(true);
          setConnectionInfo({
            ip,
            rosbridgePort,
            videoPort,
          });
          resolve();
        });
      };

      ws.onmessage = (event) => {
        handleWebSocketMessage(event);
      };

      ws.onerror = (error) => {
        console.error('WebSocket error:', error);
        finish(() => {
          setIsConnected(false);
          setConnectionInfo({ ip: '', rosbridgePort: '', videoPort: '' });
          reject(new Error('WebSocket connection failed'));
        });
      };

      ws.onclose = () => {
        console.log('WebSocket closed');
        if (!settled) {
          finish(() => {
            setIsConnected(false);
            setConnectionInfo({ ip: '', rosbridgePort: '', videoPort: '' });
            wsRef.current = null;
            reject(new Error('WebSocket connection closed before it was established'));
          });
          return;
        }
        setIsConnected(false);
        setConnectionInfo({ ip: '', rosbridgePort: '', videoPort: '' });
        wsRef.current = null;
      };

      connectTimeoutRef.current = setTimeout(() => {
        if (!settled) {
          try {
            ws.close();
          } catch (closeError) {
            console.warn('Failed to close timed out WebSocket:', closeError);
          }
          finish(() => {
            setIsConnected(false);
            setConnectionInfo({ ip: '', rosbridgePort: '', videoPort: '' });
            wsRef.current = null;
            reject(new Error('Connection timed out'));
          });
        }
      }, 8000);
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

  const advertiseTopic = (topicName, topicType) => {
    if (!wsRef.current || wsRef.current.readyState !== WebSocket.OPEN) return;
    wsRef.current.send(JSON.stringify({
      op: 'advertise',
      topic: topicName,
      type: topicType,
    }));
    console.log('Advertised', topicName);
  };

  const unadvertiseTopic = (topicName) => {
    if (!wsRef.current || wsRef.current.readyState !== WebSocket.OPEN) return;
    wsRef.current.send(JSON.stringify({
      op: 'unadvertise',
      topic: topicName,
    }));
    console.log('Unadvertised', topicName);
  };

  const publishMessage = (topicName, msgObj) => {
    if (!wsRef.current || wsRef.current.readyState !== WebSocket.OPEN) return;
    wsRef.current.send(JSON.stringify({
      op: 'publish',
      topic: topicName,
      msg: msgObj,
    }));
    console.log('Published message to', topicName);
  };

  const disconnect = () => {
    if (connectTimeoutRef.current) {
      clearTimeout(connectTimeoutRef.current);
      connectTimeoutRef.current = null;
    }

    if (topicTypeRequestRef.current) {
      const { timeoutId, handleResponse } = topicTypeRequestRef.current;
      clearTimeout(timeoutId);
      if (wsRef.current) {
        wsRef.current.removeEventListener('message', handleResponse);
      }
      topicTypeRequestRef.current = null;
    }

    if (wsRef.current) {
      wsRef.current.close();
      wsRef.current = null;
    }
    setIsConnected(false);
    setSubscribedTopics([]);
    setAvailableTopics([]);
    setConnectionInfo({ ip: '', rosbridgePort: '', videoPort: '' });
  };

  const value = {
    connectionInfo,
    subscribedTopics,
    availableTopics,
    isConnected,
    connectToROS,
    subscribeToTopic,
    unsubscribeFromTopic,
    disconnect,
    discoverTopics,
    getTopicType,
    advertiseTopic,
    unadvertiseTopic,
    publishMessage,
    ws: wsRef.current,
  };

  return <ROSContext.Provider value={value}>{children}</ROSContext.Provider>;
};