import React, { useState, useEffect, useLayoutEffect } from 'react';
import { View, Text, ScrollView, TouchableOpacity, StyleSheet, Dimensions } from 'react-native';
import { useROS } from '../context/ROSContext';
import VideoPanel from '../components/VideoPanel';
import { theme } from '../theme/colors';

export default function DashboardScreen({ navigation }) {
  const { subscribedTopics, connectionInfo, isConnected, unsubscribeFromTopic } = useROS();
  const [isLandscape, setIsLandscape] = useState(false);

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

  const renderTopics = () => (
    <View style={styles.section}>
      <Text style={styles.sectionTitle}>SUBSCRIBED TOPICS</Text>
      
      {subscribedTopics.length === 0 ? (
        <View style={styles.emptyContainer}>
          <Text style={styles.emptyText}>No topics subscribed yet</Text>
          <Text style={styles.emptySubtext}>Tap "Add Topic" below to start monitoring</Text>
        </View>
      ) : (
        <View>
          {subscribedTopics.map((item, index) => (
            <View key={index} style={styles.topicWidget}>
              <View style={styles.topicHeader}>
                <View style={styles.topicTitleContainer}>
                  <Text style={styles.topicName}>{item.topic}</Text>
                  <Text style={styles.topicType}>{item.type}</Text>
                </View>
                <TouchableOpacity 
                  onPress={() => unsubscribeFromTopic(item.topic)}
                  style={styles.unsubscribeButton}
                >
                  <Text style={styles.unsubscribeText}>✕</Text>
                </TouchableOpacity>
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
          ))}
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

          {isLandscape && (
            <TouchableOpacity
              onPress={() => navigation.navigate('TopicBrowser')}
              style={styles.headerIconButton}
              accessibilityRole="button"
              accessibilityLabel="Add Topic"
            >
              <Text style={styles.headerIconText}>+</Text>
            </TouchableOpacity>
          )}
        </View>
      </View>

      <ScrollView style={styles.scrollContainer}>
        {isLandscape ? (
          <View style={styles.landscapeContainer}>
            <View style={styles.landscapeLeft}>
              <VideoPanel />
            </View>
            <View style={styles.landscapeRight}>
              {renderTopics()}
            </View>
          </View>
        ) : (
          <View style={styles.content}>
            <VideoPanel />
            {renderTopics()}
          </View>
        )}
      </ScrollView>

      {!isLandscape && (
        <View style={styles.buttonContainer}>
          <TouchableOpacity
            style={styles.addButton}
            onPress={() => navigation.navigate('TopicBrowser')}
          >
            <Text style={styles.addButtonText}>+ ADD TOPIC</Text>
          </TouchableOpacity>
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
    paddingTop: 30,
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
    flexDirection: 'row',
    padding: 16,
  },
  landscapeLeft: {
    flex: 1,
    marginRight: 8,
  },
  landscapeRight: {
    flex: 1,
    marginLeft: 8,
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
  addButton: {
    backgroundColor: theme.accent.primary,
    borderRadius: 8,
    padding: 16,
    alignItems: 'center',
  },
  addButtonText: {
    color: '#FFFFFF',
    fontSize: 16,
    fontWeight: '700',
    letterSpacing: 1,
  },
  headerIconButton: {
    paddingHorizontal: 12,
    paddingVertical: 6,
  },
  headerIconText: {
    fontSize: 22,
    fontWeight: '700',
    color: theme.text.primary,
  },
});