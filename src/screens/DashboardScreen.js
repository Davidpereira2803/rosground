import React from 'react';
import { View, Text, ScrollView, TouchableOpacity, StyleSheet } from 'react-native';
import { useROS } from '../context/ROSContext';
import VideoPanel from '../components/VideoPanel';
import { theme } from '../theme/colors';

export default function DashboardScreen({ navigation }) {
  const { subscribedTopics, connectionInfo, isConnected, unsubscribeFromTopic } = useROS();

  return (
    <View style={styles.container}>
      <View style={styles.header}>
        <View style={styles.headerRow}>
          <Text style={styles.headerTitle}>Dashboard</Text>
          <View style={styles.statusBadge}>
            <View style={[styles.statusDot, { backgroundColor: isConnected ? theme.status.connected : theme.status.disconnected }]} />
            <Text style={styles.statusText}>
              {isConnected ? 'Connected' : 'Disconnected'}
            </Text>
          </View>
        </View>
        <Text style={styles.headerSubtext}>
          {connectionInfo.ip}:{connectionInfo.rosbridgePort}
        </Text>
      </View>

      <ScrollView style={styles.scrollContainer}>
        <View style={styles.content}>
          <VideoPanel />

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
        </View>
      </ScrollView>

      <View style={styles.buttonContainer}>
        <TouchableOpacity
          style={styles.addButton}
          onPress={() => navigation.navigate('TopicBrowser')}
        >
          <Text style={styles.addButtonText}>+ ADD TOPIC</Text>
        </TouchableOpacity>
      </View>
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
    padding: 20,
    borderBottomWidth: 2,
    borderBottomColor: theme.border.primary,
  },
  headerRow: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    marginBottom: 8,
  },
  headerTitle: {
    fontSize: 24,
    fontWeight: '700',
    color: theme.text.primary,
  },
  statusBadge: {
    flexDirection: 'row',
    alignItems: 'center',
    backgroundColor: theme.background.card,
    paddingHorizontal: 12,
    paddingVertical: 6,
    borderRadius: 16,
    borderWidth: 1,
    borderColor: theme.border.subtle,
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
  headerSubtext: {
    fontSize: 13,
    color: theme.text.muted,
  },
  scrollContainer: {
    flex: 1,
  },
  content: {
    padding: 16,
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
    padding: 16,
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
});