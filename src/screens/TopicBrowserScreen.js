import React, { useState } from 'react';
import { View, Text, TextInput, TouchableOpacity, StyleSheet, Alert, ScrollView } from 'react-native';
import { useROS } from '../context/ROSContext';
import { theme } from '../theme/colors';

export default function TopicBrowserScreen({ navigation }) {
  const { subscribeToTopic, isConnected } = useROS();
  const [topicName, setTopicName] = useState('');
  const [topicType, setTopicType] = useState('');

  const handleSubscribe = () => {
    if (!topicName.trim()) {
      Alert.alert('Error', 'Please enter a topic name');
      return;
    }

    if (!topicType.trim()) {
      Alert.alert('Error', 'Please enter a topic type');
      return;
    }

    if (!isConnected) {
      Alert.alert('Error', 'Not connected to ROS');
      return;
    }

    subscribeToTopic(topicName.trim(), topicType.trim());
    navigation.goBack();
  };

  return (
    <View style={styles.container}>
      <View style={styles.header}>
        <Text style={styles.subtitle}>
          Manually enter the topic name and type to subscribe
        </Text>
      </View>

      <ScrollView
        style={styles.scroll}
        contentContainerStyle={styles.scrollContent}
        keyboardShouldPersistTaps="handled"
      >
        <View style={styles.formContainer}>
          <View style={styles.inputContainer}>
            <Text style={styles.label}>TOPIC NAME</Text>
            <TextInput
              style={styles.input}
              value={topicName}
              onChangeText={setTopicName}
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
    padding: 20,
    paddingTop: 40,
    backgroundColor: theme.background.secondary,
    borderBottomWidth: 2,
    borderBottomColor: theme.border.primary,
  },
  title: {
    fontSize: 28,
    fontWeight: '700',
    color: theme.text.primary,
    marginBottom: 8,
  },
  subtitle: {
    fontSize: 14,
    color: theme.text.secondary,
    lineHeight: 20,
    textAlign: 'center',
  },
  scroll: {
    flex: 1,
  },
  scrollContent: {
    paddingBottom: 24,
  },
  formContainer: {
    padding: 20,
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
});