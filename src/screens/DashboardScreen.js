import React, { useState, useEffect, useLayoutEffect } from 'react';
import { View, Text, ScrollView, TouchableOpacity, StyleSheet, Dimensions } from 'react-native';
import { useROS } from '../context/ROSContext';
import VideoPanel from '../components/VideoPanel';
import { theme } from '../theme/colors';
import Constants from 'expo-constants';

const ENABLE_VIDEO = false;

export default function DashboardScreen({ navigation }) {
  const { subscribedTopics, connectionInfo, isConnected, unsubscribeFromTopic } = useROS();
  const [isLandscape, setIsLandscape] = useState(false);

  const isDev = __DEV__ || Constants.appOwnership === 'expo';

const hasVideoStream = ENABLE_VIDEO && isConnected && connectionInfo.ip && connectionInfo.videoPort;

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
  const showDevPreview = isDev && !isConnected && subscribedTopics.length === 0;

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
          <Text style={styles.topicName} numberOfLines={2} ellipsizeMode="tail">
            {item.topic}
          </Text>
          <View style={styles.topicMetaRow}>
            <View style={styles.topicTypePill}>
              <Text style={styles.topicTypePillText} numberOfLines={1} ellipsizeMode="tail">
                {item.type}
              </Text>
            </View>
          </View>
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
        {item.lastMsg ? (
          <View style={styles.messageViewport}>
            <ScrollView
              style={styles.messageScroll}
              contentContainerStyle={styles.messageScrollContent}
              nestedScrollEnabled
              showsVerticalScrollIndicator={true}
              showsHorizontalScrollIndicator={true}
              scrollEventThrottle={16}
            >
              <Text style={styles.messageText} selectable>
                {JSON.stringify(item.lastMsg, null, 2)}
              </Text>
            </ScrollView>
          </View>
        ) : (
          <View style={styles.emptyMessageState}>
            <Text style={styles.emptyMessageText}>Waiting for the first message...</Text>
          </View>
        )}
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
          {hasVideoStream ? (
            <View style={styles.landscapeLeft}>
              <VideoPanel />
            </View>
          ) : null}
          <View style={styles.landscapeRight}>
            {showDevPreview && (
              <View style={styles.previewBanner}>
                <Text style={styles.previewBannerText}>
                  Preview mode is showing sample topics until you connect to a live ROS system.
                </Text>
              </View>
            )}
            <View style={styles.sectionHeaderRow}>
              <Text style={styles.sectionTitle}>SUBSCRIBED TOPICS</Text>
              <Text style={styles.sectionCount}>{displayTopics.length}</Text>
            </View>
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
                nestedScrollEnabled
              >
                {displayTopics.map((item, index) => renderTopicCard(item, index))}
              </ScrollView>
            )}
          </View>
        </View>
      ) : (
        <ScrollView style={styles.scrollContainer} nestedScrollEnabled>
          <View style={styles.content}>
            {ENABLE_VIDEO && <VideoPanel />}
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
    color: theme.text.accent,
    letterSpacing: 1,
  },
  sectionHeaderRow: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    marginBottom: 16,
  },
  sectionCount: {
    minWidth: 28,
    textAlign: 'center',
    paddingHorizontal: 8,
    paddingVertical: 4,
    borderRadius: 12,
    overflow: 'hidden',
    backgroundColor: theme.background.card,
    color: theme.text.secondary,
    fontSize: 12,
    fontWeight: '700',
  },
  emptyContainer: {
    alignItems: 'center',
    paddingVertical: 40,
    paddingHorizontal: 16,
    backgroundColor: theme.background.card,
    borderRadius: 12,
    borderWidth: 1,
    borderColor: theme.border.primary,
  },
  emptyText: {
    fontSize: 16,
    color: theme.text.secondary,
    marginBottom: 8,
    textAlign: 'center',
  },
  emptySubtext: {
    fontSize: 13,
    color: theme.text.muted,
    textAlign: 'center',
  },
  previewBanner: {
    backgroundColor: theme.background.card,
    borderColor: theme.border.primary,
    borderWidth: 1,
    borderRadius: 10,
    padding: 12,
    marginBottom: 14,
  },
  previewBannerText: {
    color: theme.text.secondary,
    fontSize: 12,
    lineHeight: 17,
  },
  topicWidget: {
    backgroundColor: theme.background.card,
    padding: 16,
    borderRadius: 12,
    marginBottom: 12,
    borderWidth: 1,
    borderColor: theme.border.primary,
    shadowColor: '#000',
    shadowOpacity: 0.12,
    shadowRadius: 8,
    shadowOffset: { width: 0, height: 3 },
    elevation: 2,
  },
  topicWidgetLandscape: {
    width: 320,
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
    paddingRight: 12,
  },
  topicName: {
    fontSize: 16,
    fontWeight: '700',
    color: theme.text.primary,
    marginBottom: 4,
    lineHeight: 20,
  },
  topicMetaRow: {
    flexDirection: 'row',
    alignItems: 'center',
    flexWrap: 'wrap',
    gap: 8,
  },
  topicTypePill: {
    alignSelf: 'flex-start',
    backgroundColor: theme.background.primary,
    borderWidth: 1,
    borderColor: theme.border.primary,
    borderRadius: 999,
    paddingHorizontal: 10,
    paddingVertical: 4,
    maxWidth: '100%',
  },
  topicTypePillText: {
    fontSize: 10,
    color: theme.text.secondary,
    fontWeight: '600',
    letterSpacing: 0.3,
  },
  unsubscribeButton: {
    width: 32,
    height: 32,
    borderRadius: 16,
    alignItems: 'center',
    justifyContent: 'center',
    backgroundColor: theme.background.primary,
    borderWidth: 1,
    borderColor: theme.border.primary,
    marginLeft: 12,
  },
  unsubscribeText: {
    fontSize: 18,
    color: theme.accent.error,
    fontWeight: '700',
    lineHeight: 20,
  },
  messageContainer: {
    marginTop: 4,
  },
  messageLabel: {
    fontSize: 10,
    fontWeight: '600',
    marginBottom: 8,
    color: theme.text.accent,
    letterSpacing: 1,
  },
  messageViewport: {
    maxHeight: 190,
    borderRadius: 8,
    borderWidth: 1,
    borderColor: theme.border.subtle,
    backgroundColor: theme.background.primary,
    overflow: 'hidden',
  },
  messageScroll: {
    maxHeight: 190,
  },
  messageScrollContent: {
    paddingBottom: 10,
  },
  messageText: {
    fontSize: 11,
    lineHeight: 16,
    fontFamily: 'monospace',
    color: theme.accent.success,
    paddingHorizontal: 12,
    paddingVertical: 10,
    minWidth: '100%',
  },
  emptyMessageState: {
    borderRadius: 8,
    borderWidth: 1,
    borderColor: theme.border.subtle,
    backgroundColor: theme.background.primary,
    paddingHorizontal: 12,
    paddingVertical: 10,
  },
  emptyMessageText: {
    color: theme.text.muted,
    fontSize: 11,
    lineHeight: 16,
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