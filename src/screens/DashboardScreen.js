import React, { useState, useEffect, useLayoutEffect } from 'react';
import { View, Text, ScrollView, TouchableOpacity, StyleSheet, Dimensions } from 'react-native';
import { useROS } from '../context/ROSContext';
import VideoPanel from '../components/VideoPanel';
import { theme } from '../theme/colors';
import Constants from 'expo-constants';

export default function DashboardScreen({ navigation }) {
  const { subscribedTopics, connectionInfo, isConnected, unsubscribeFromTopic } = useROS();
  const [isLandscape, setIsLandscape] = useState(false);

  const isDev = __DEV__ || Constants.appOwnership === 'expo';

  const hasVideoStream = isConnected && connectionInfo.ip && connectionInfo.videoPort;

  const mockTopics = isDev ? [
    {
      topic: '/battery',
      type: 'std_msgs/msg/Float32',
      lastMsg: { data: 85.5 }
    },
    {
      topic: '/cmd_vel',
      type: 'geometry_msgs/msg/Twist',
      lastMsg: {
        linear: { x: 0.5, y: 0.0, z: 0.0 },
        angular: { x: 0.0, y: 0.0, z: 0.2 }
      }
    },
    {
      topic: '/odom',
      type: 'nav_msgs/msg/Odometry',
      lastMsg: {
        pose: {
          position: { x: 1.234, y: 5.678, z: 0.0 },
          orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
        }
      }
    }
  ] : [];

  const displayTopics = isDev && subscribedTopics.length === 0 
    ? mockTopics 
    : subscribedTopics;

  useEffect(() => {
    const updateOrientation = () => {
      const { width, height } = Dimensions.get('window');
      setIsLandscape(width > height);
    };

    updateOrientation();
    const subscription = Dimensions.addEventListener('change', updateOrientation);
    return () => {
      subscription?.remove();
    };
  }, []);

  useLayoutEffect(() => {
    navigation.setOptions({
      headerRight: isLandscape
        ? () => (
            <TouchableOpacity
              onPress={() => navigation.navigate('TopicBrowser')}
              style={styles.headerIconButton}
              accessibilityRole="button"
              accessibilityLabel="Add Topic"
            >
              <Text style={styles.headerIconText}>+</Text>
            </TouchableOpacity>
          )
        : undefined,
    });
  }, [navigation, isLandscape]);

  const renderTopicCard = (item, index) => (
    <View key={index} style={[styles.topicWidget, isLandscape && styles.topicWidgetLandscape]}>
      <View style={styles.topicHeader}>
        <View style={styles.topicTitleContainer}>
          <Text style={styles.topicName}>{item.topic}</Text>
          <Text style={styles.topicType}>{item.type}</Text>
        </View>
        {!isDev || subscribedTopics.length > 0 ? (
          <TouchableOpacity 
            onPress={() => unsubscribeFromTopic(item.topic)}
            style={styles.unsubscribeButton}
          >
            <Text style={styles.unsubscribeText}>✕</Text>
          </TouchableOpacity>
        ) : (
          <View style={styles.devBadge}>
            <Text style={styles.devBadgeText}>MOCK</Text>
          </View>
        )}
      </View>
      <View style={styles.messageContainer}>
        <Text style={styles.messageLabel}>LATEST MESSAGE</Text>
        <ScrollView horizontal style={styles.messageScroll}>
          <Text style={styles.messageText}>
            {item.lastMsg ? JSON.stringify(item.lastMsg, null, 2) : 'Waiting for data...'}
          </Text>
        </ScrollView>
      </View>
    </View>
  );

  const renderTopics = () => (
    <View style={styles.section}>
      <Text style={styles.sectionTitle}>SUBSCRIBED TOPICS</Text>
      
      {displayTopics.length === 0 ? (
        <View style={styles.emptyContainer}>
          <Text style={styles.emptyText}>No topics subscribed yet</Text>
          <Text style={styles.emptySubtext}>Tap "Add Topic" below to start monitoring</Text>
        </View>
      ) : (
        <View>
          {displayTopics.map((item, index) => renderTopicCard(item, index))}
        </View>
      )}
    </View>
  );

  return (
    <View style={styles.container}>
      <View style={styles.header}>
        <View style={styles.headerRow}>
          <View style={styles.headerLeftRow}>
            <View style={styles.statusBadge}>
              <View style={[styles.statusDot, { backgroundColor: isConnected ? theme.status.connected : theme.status.disconnected }]} />
              <Text style={styles.statusText}>
                {isConnected ? 'Connected' : 'Disconnected'}
              </Text>
            </View>
            <Text style={styles.headerConnectionText}>
              {connectionInfo.ip}:{connectionInfo.rosbridgePort}
            </Text>
          </View>

          {/* Right: icon actions in landscape */}
          {isLandscape && (
            <View style={styles.headerActions}>
              <TouchableOpacity
                onPress={() => navigation.navigate('TopicBrowser')}
                style={styles.headerIconButton}
                accessibilityRole="button"
                accessibilityLabel="Add Topic"
                hitSlop={{ top: 8, bottom: 8, left: 8, right: 8 }}
              >
                <Text style={styles.headerIconText}>+</Text>
              </TouchableOpacity>

              <TouchableOpacity
                onPress={() => navigation.navigate('Publish')}
                style={[styles.headerIconButton, styles.headerIconButtonRight]}
                accessibilityRole="button"
                accessibilityLabel="Publish Test"
                hitSlop={{ top: 8, bottom: 8, left: 8, right: 8 }}
              >
                <Text style={styles.headerIconText}>↑</Text>
              </TouchableOpacity>
            </View>
          )}
        </View>
      </View>

      {isLandscape ? (
        <View style={styles.landscapeContainer}>
          <View style={[
            styles.landscapeLeft, 
            !hasVideoStream && styles.landscapeLeftCompact
          ]}>
            <VideoPanel />
          </View>
          <View style={styles.landscapeRight}>
            <Text style={styles.sectionTitle}>SUBSCRIBED TOPICS</Text>
            {displayTopics.length === 0 ? (
              <View style={styles.emptyContainer}>
                <Text style={styles.emptyText}>No topics subscribed yet</Text>
                <Text style={styles.emptySubtext}>Tap "+" to start monitoring</Text>
              </View>
            ) : (
              <ScrollView 
                horizontal 
                showsHorizontalScrollIndicator={true}
                style={styles.topicsHorizontalScroll}
                contentContainerStyle={styles.topicsHorizontalContent}
              >
                {displayTopics.map((item, index) => renderTopicCard(item, index))}
              </ScrollView>
            )}
          </View>
        </View>
      ) : (
        <ScrollView style={styles.scrollContainer}>
          <View style={styles.content}>
            <VideoPanel />
            {renderTopics()}
          </View>
        </ScrollView>
      )}

      {/* Bottom actions: only in portrait */}
      {!isLandscape && (
        <View style={styles.buttonContainer}>
          <View style={styles.buttonRow}>
            <TouchableOpacity
              style={[styles.actionButton, styles.buttonLeft]}
              onPress={() => navigation.navigate('TopicBrowser')}
            >
              <Text style={styles.actionButtonText}>ADD TOPIC</Text>
            </TouchableOpacity>

            <TouchableOpacity
              style={[styles.actionButton, styles.buttonRight]}
              onPress={() => navigation.navigate('Publish')}
            >
              <Text style={styles.actionButtonText}>PUBLISH TEST</Text>
            </TouchableOpacity>
          </View>
        </View>
      )}
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: theme.background.primary,
  },
  header: {
    backgroundColor: theme.background.secondary,
    paddingHorizontal: 16,
    paddingVertical: 12,
    paddingTop: 40,
    borderBottomWidth: 2,
    borderBottomColor: theme.border.primary,
  },
  headerRow: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
  },
  headerLeftRow: {
    flexDirection: 'row',
    alignItems: 'center',
    flexShrink: 1,
  },
  statusBadge: {
    flexDirection: 'row',
    alignItems: 'center',
    backgroundColor: theme.background.card,
    paddingHorizontal: 10,
    paddingVertical: 4,
    borderRadius: 14,
    borderWidth: 1,
    borderColor: theme.border.subtle,
    marginRight: 10,
  },
  statusDot: {
    width: 8,
    height: 8,
    borderRadius: 4,
    marginRight: 6,
  },
  statusText: {
    fontSize: 12,
    color: theme.text.secondary,
    fontWeight: '600',
  },
  headerConnectionText: {
    fontSize: 12,
    color: theme.text.muted,
  },
  scrollContainer: {
    flex: 1,
  },
  content: {
    padding: 16,
  },
  landscapeContainer: {
    flex: 1,
    flexDirection: 'row',
    padding: 16,
  },
  landscapeLeft: {
    width: '40%',
    marginRight: 16,
  },
  landscapeLeftCompact: {
    width: '25%',
  },
  landscapeRight: {
    flex: 1,
  },
  topicsHorizontalScroll: {
    flex: 1,
  },
  topicsHorizontalContent: {
    paddingRight: 16,
  },
  section: {
    marginTop: 8,
  },
  sectionTitle: {
    fontSize: 14,
    fontWeight: '700',
    marginBottom: 16,
    color: theme.text.accent,
    letterSpacing: 1,
  },
  emptyContainer: {
    alignItems: 'center',
    paddingVertical: 40,
  },
  emptyText: {
    fontSize: 16,
    color: theme.text.secondary,
    marginBottom: 8,
  },
  emptySubtext: {
    fontSize: 13,
    color: theme.text.muted,
  },
  topicWidget: {
    backgroundColor: theme.background.card,
    padding: 16,
    borderRadius: 12,
    marginBottom: 12,
    borderWidth: 1,
    borderColor: theme.border.primary,
  },
  topicWidgetLandscape: {
    width: 300,
    marginRight: 12,
    marginBottom: 0,
  },
  topicHeader: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'flex-start',
    marginBottom: 12,
  },
  topicTitleContainer: {
    flex: 1,
  },
  topicName: {
    fontSize: 16,
    fontWeight: '700',
    color: theme.text.primary,
    marginBottom: 4,
  },
  topicType: {
    fontSize: 11,
    color: theme.text.muted,
  },
  unsubscribeButton: {
    padding: 4,
    marginLeft: 12,
  },
  unsubscribeText: {
    fontSize: 20,
    color: theme.accent.error,
  },
  messageContainer: {
    marginTop: 8,
  },
  messageLabel: {
    fontSize: 10,
    fontWeight: '600',
    marginBottom: 8,
    color: theme.text.accent,
    letterSpacing: 1,
  },
  messageScroll: {
    maxHeight: 120,
  },
  messageText: {
    fontSize: 11,
    color: theme.accent.success,
    backgroundColor: theme.background.primary,
    padding: 12,
    borderRadius: 6,
    borderWidth: 1,
    borderColor: theme.border.subtle,
  },
  buttonContainer: {
    padding: 18,
    paddingBottom: 30,
    backgroundColor: theme.background.secondary,
    borderTopWidth: 2,
    borderTopColor: theme.border.primary,
  },
  buttonRow: {
    flexDirection: 'row',
  },
  actionButton: {
    flex: 1,
    backgroundColor: theme.accent.primary,
    borderRadius: 8,
    padding: 16,
    alignItems: 'center',
  },
  buttonLeft: { marginRight: 8 },
  buttonRight: { marginLeft: 8 },
  actionButtonText: {
    color: '#FFFFFF',
    fontSize: 16,
    fontWeight: '700',
    letterSpacing: 1,
  },
  headerActions: {
    flexDirection: 'row',
    alignItems: 'center',
  },
  headerIconButton: {
    paddingHorizontal: 12,
    paddingVertical: 6,
  },
  headerIconButtonRight: {
    marginLeft: 8,
  },
  headerIconText: {
    fontSize: 22,
    fontWeight: '700',
    color: theme.text.primary,
  },
  devBadge: {
    backgroundColor: theme.accent.warning,
    paddingHorizontal: 8,
    paddingVertical: 2,
    borderRadius: 4,
  },
  devBadgeText: {
    color: '#000',
    fontSize: 9,
    fontWeight: '700',
    letterSpacing: 0.5,
  },
});