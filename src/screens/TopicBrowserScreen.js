import React, { useState, useEffect } from 'react';
import { View, Text, TextInput, TouchableOpacity, StyleSheet, ScrollView, FlatList } from 'react-native';
import { useROS } from '../context/ROSContext';
import { theme } from '../theme/colors';

export default function TopicBrowserScreen({ navigation }) {
  const { subscribeToTopic, isConnected, availableTopics, discoverTopics, getTopicType } = useROS();
  const [topicName, setTopicName] = useState('');
  const [topicType, setTopicType] = useState('');
  const [showAutoDetect, setShowAutoDetect] = useState(true);
  const [filteredTopics, setFilteredTopics] = useState([]);
  const [message, setMessage] = useState('');
  const [messageType, setMessageType] = useState('info');

  useEffect(() => {
    if (isConnected && availableTopics.length === 0) {
      discoverTopics();
    }
  }, [isConnected]);

  useEffect(() => {
    if (topicName.trim()) {
      setFilteredTopics(
        availableTopics.filter(topic => 
          topic.toLowerCase().includes(topicName.toLowerCase())
        )
      );
    } else {
      setFilteredTopics(availableTopics);
    }
  }, [topicName, availableTopics]);

  useEffect(() => {
    setMessage('');
    setMessageType('info');
  }, [topicName, topicType]);

  const handleTopicSelect = (topic) => {
    setTopicName(topic);
    setShowAutoDetect(false);
    setMessage('');
    
    getTopicType(topic, (type) => {
      setTopicType(type);
      setMessage(type ? `Loaded type for ${topic}.` : `No type returned for ${topic}.`);
      setMessageType(type ? 'success' : 'error');
    });
  };

  const handleSubscribe = () => {
    const trimmedTopicName = topicName.trim();
    const trimmedTopicType = topicType.trim();

    if (!trimmedTopicName) {
      setMessage('Please enter a topic name.');
      setMessageType('error');
      return;
    }

    if (!trimmedTopicType) {
      setMessage('Please enter a topic type.');
      setMessageType('error');
      return;
    }

    if (!isConnected) {
      setMessage('Connect to ROS before subscribing.');
      setMessageType('error');
      return;
    }

    if (!trimmedTopicName.startsWith('/')) {
      setMessage('Topic names should start with /.');
      setMessageType('error');
      return;
    }

    subscribeToTopic(trimmedTopicName, trimmedTopicType);
    setMessage(`Subscribed to ${trimmedTopicName}.`);
    setMessageType('success');
    navigation.goBack();
  };

  const handleRefresh = () => {
    discoverTopics();
    setMessage('Refreshing available topics...');
    setMessageType('info');
  };

  return (
    <View style={styles.container}>
      <View style={styles.header}>
        <Text style={styles.title}>Add Topic</Text>
        <Text style={styles.subtitle}>Select from available topics or enter manually</Text>
      </View>

      <ScrollView
        style={styles.scroll}
        contentContainerStyle={styles.scrollContent}
        keyboardShouldPersistTaps="handled"
      >
        <View style={styles.formContainer}>
          {/* Auto-detect section */}
          {isConnected && (
            <View style={styles.autoDetectSection}>
              <View style={styles.autoDetectHeader}>
                <Text style={styles.sectionTitle}>AVAILABLE TOPICS ({availableTopics.length})</Text>
                <TouchableOpacity onPress={handleRefresh} style={styles.refreshButton}>
                  <Text style={styles.refreshText}>↻ Refresh</Text>
                </TouchableOpacity>
              </View>

              {showAutoDetect && filteredTopics.length > 0 && (
                <View style={styles.topicList}>
                  <FlatList
                    data={filteredTopics}
                    keyExtractor={(item) => item}
                    nestedScrollEnabled
                    showsVerticalScrollIndicator={true}
                    renderItem={({ item }) => (
                      <TouchableOpacity
                        style={styles.topicItem}
                        onPress={() => handleTopicSelect(item)}
                      >
                        <Text style={styles.topicItemText}>{item}</Text>
                      </TouchableOpacity>
                    )}
                  />
                </View>
              )}

              {availableTopics.length === 0 && (
                <Text style={styles.noTopicsText}>
                  No topics detected. Make sure rosapi is running.
                </Text>
              )}
            </View>
          )}

          {/* Manual input section */}
          <Text style={styles.sectionTitle}>MANUAL ENTRY</Text>

          <View style={styles.inputContainer}>
            <Text style={styles.label}>TOPIC NAME</Text>
            <TextInput
              style={styles.input}
              value={topicName}
              onChangeText={(text) => {
                setTopicName(text);
                setShowAutoDetect(true);
              }}
              placeholder="e.g. /battery"
              placeholderTextColor={theme.text.placeholder}
              autoCapitalize="none"
              autoCorrect={false}
            />
            <Text style={styles.hint}>Example: /battery, /cmd_vel, /odom</Text>
          </View>

          <View style={styles.inputContainer}>
            <Text style={styles.label}>TOPIC TYPE</Text>
            <TextInput
              style={styles.input}
              value={topicType}
              onChangeText={setTopicType}
              placeholder="e.g. std_msgs/msg/Float32"
              placeholderTextColor={theme.text.placeholder}
              autoCapitalize="none"
              autoCorrect={false}
            />
            <Text style={styles.hint}>
              Example: std_msgs/msg/Float32, geometry_msgs/msg/Twist
            </Text>
          </View>

          <TouchableOpacity
            style={[styles.button, !isConnected && styles.buttonDisabled]}
            onPress={handleSubscribe}
            disabled={!isConnected}
          >
            <Text style={styles.buttonText}>SUBSCRIBE</Text>
          </TouchableOpacity>

          {message ? (
            <View style={[styles.messageBanner, messageType === 'error' && styles.messageBannerError, messageType === 'success' && styles.messageBannerSuccess]}>
              <Text style={styles.messageBannerText}>{message}</Text>
            </View>
          ) : null}

          {!isConnected && (
            <View style={styles.warningContainer}>
              <Text style={styles.warningText}>
                ⚠️ Not connected to ROS. Please connect first.
              </Text>
            </View>
          )}
        </View>
      </ScrollView>

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
  title: {
    fontSize: 18,
    fontWeight: '700',
    color: theme.text.primary,
    marginBottom: 4,
  },
  subtitle: {
    fontSize: 12,
    color: theme.text.muted,
  },
  scroll: {
    flex: 1,
  },
  scrollContent: {
    paddingBottom: 40, // space above footer
  },
  formContainer: {
    padding: 20,
  },
  autoDetectSection: {
    marginBottom: 24,
  },
  autoDetectHeader: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    marginBottom: 12,
  },
  sectionTitle: {
    fontSize: 12,
    fontWeight: '700',
    color: theme.text.accent,
    letterSpacing: 1,
    marginBottom: 12,
  },
  refreshButton: {
    padding: 6,
  },
  refreshText: {
    fontSize: 14,
    color: theme.accent.primary,
    fontWeight: '600',
  },
  topicList: {
    backgroundColor: theme.background.card,
    borderRadius: 8,
    borderWidth: 1,
    borderColor: theme.border.primary,
    maxHeight: 300,
    overflow: 'hidden',
  },
  topicItem: {
    padding: 12,
    borderBottomWidth: 1,
    borderBottomColor: theme.border.subtle,
    backgroundColor: theme.background.card,
  },
  topicItemText: {
    fontSize: 14,
    color: theme.text.primary,
  },
  noTopicsText: {
    fontSize: 13,
    color: theme.text.muted,
    fontStyle: 'italic',
    textAlign: 'center',
    padding: 16,
  },
  inputContainer: {
    marginBottom: 24,
  },
  label: {
    fontSize: 12,
    fontWeight: '600',
    marginBottom: 8,
    color: theme.text.accent,
    letterSpacing: 1,
  },
  input: {
    backgroundColor: theme.background.input,
    borderWidth: 1,
    borderColor: theme.border.primary,
    borderRadius: 8,
    padding: 14,
    fontSize: 16,
    color: theme.text.primary,
    marginBottom: 6,
  },
  hint: {
    fontSize: 12,
    color: theme.text.muted,
    fontStyle: 'italic',
  },
  button: {
    backgroundColor: theme.accent.primary,
    borderRadius: 8,
    padding: 16,
    alignItems: 'center',
    marginTop: 12,
  },
  buttonDisabled: {
    backgroundColor: theme.accent.secondary,
    opacity: 0.5,
  },
  buttonText: {
    color: '#FFFFFF',
    fontSize: 16,
    fontWeight: '700',
    letterSpacing: 1,
  },
  warningContainer: {
    marginTop: 20,
    padding: 12,
    backgroundColor: theme.background.card,
    borderRadius: 8,
    borderWidth: 1,
    borderColor: theme.accent.warning,
  },
  warningText: {
    color: theme.accent.warning,
    textAlign: 'center',
    fontSize: 14,
  },
  messageBanner: {
    marginTop: 14,
    padding: 12,
    borderRadius: 8,
    backgroundColor: theme.background.card,
    borderWidth: 1,
    borderColor: theme.border.primary,
  },
  messageBannerError: {
    borderColor: theme.accent.error,
  },
  messageBannerSuccess: {
    borderColor: theme.accent.success,
  },
  messageBannerText: {
    color: theme.text.primary,
    fontSize: 13,
    textAlign: 'center',
    lineHeight: 18,
  },
  footer: {
    padding: 14,
    paddingHorizontal: 20,
    paddingBottom: 30,
    paddingVertical: 20,
    borderTopWidth: 2,
    borderTopColor: theme.border.primary,
    backgroundColor: theme.background.secondary,
    alignItems: 'center',
  },
  footerText: {
    color: theme.text.primary,
    fontSize: 18,
    fontWeight: '600',
  },
});