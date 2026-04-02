import React, { useState, useEffect } from 'react';
import { View, Text, TextInput, TouchableOpacity, StyleSheet, ScrollView } from 'react-native';
import { useROS } from '../context/ROSContext';
import { theme } from '../theme/colors';

export default function PublishScreen({ navigation }) {
  const { isConnected, advertiseTopic, publishMessage, unadvertiseTopic } = useROS();
  const [topicName, setTopicName] = useState('/chatter');
  const [topicType, setTopicType] = useState('std_msgs/msg/String');
  const [rawMessage, setRawMessage] = useState('{"data":"Hello from phone"}');
  const [advertised, setAdvertised] = useState(false);
  const [message, setMessage] = useState('');
  const [messageType, setMessageType] = useState('info');

  useEffect(() => {
    if (advertised) {
      setAdvertised(false);
      setMessage('Topic details changed. Advertise again before publishing.');
      setMessageType('info');
    }
  }, [topicName, topicType, rawMessage]);

  const handleAdvertise = () => {
    if (!isConnected) {
      setMessage('Connect to rosbridge before advertising.');
      setMessageType('error');
      return;
    }
    if (!topicName.trim() || !topicType.trim()) {
      setMessage('Topic name and type are required.');
      setMessageType('error');
      return;
    }
    advertiseTopic(topicName.trim(), topicType.trim());
    setAdvertised(true);
    setMessage(`Advertising ${topicName.trim()}.`);
    setMessageType('success');
  };

  const handlePublish = () => {
    if (!advertised) {
      setMessage('Advertise the topic before publishing.');
      setMessageType('error');
      return;
    }
    try {
      const msgObj = JSON.parse(rawMessage);
      publishMessage(topicName.trim(), msgObj);
      setMessage('Message sent.');
      setMessageType('success');
    } catch (e) {
      setMessage(`Invalid JSON: ${e.message}`);
      setMessageType('error');
    }
  };

  const handleUnadvertise = () => {
    if (advertised) {
      unadvertiseTopic(topicName.trim());
      setAdvertised(false);
      setMessage(`Stopped advertising ${topicName.trim()}.`);
      setMessageType('info');
    }
  };

  return (
    <View style={styles.container}>
      <View style={styles.header}>
        <Text style={styles.title}>Publish Message</Text>
        <Text style={styles.subtitle}>Send a manual ROS message via rosbridge</Text>
      </View>

      <ScrollView style={styles.scroll} contentContainerStyle={styles.scrollContent} keyboardShouldPersistTaps="handled">
        <View style={styles.form}>
          <Text style={styles.label}>TOPIC NAME</Text>
          <TextInput
            style={styles.input}
            value={topicName}
            onChangeText={setTopicName}
            placeholder="/chatter"
            autoCapitalize="none"
          />

            <Text style={styles.label}>TOPIC TYPE</Text>
          <TextInput
            style={styles.input}
            value={topicType}
            onChangeText={setTopicType}
            placeholder="std_msgs/msg/String"
            autoCapitalize="none"
          />
          <Text style={styles.hint}>Use std_msgs/String for ROS1</Text>

          <Text style={styles.label}>MESSAGE JSON</Text>
          <TextInput
            style={[styles.input, styles.multiline]}
            value={rawMessage}
            onChangeText={setRawMessage}
            multiline
            autoCapitalize="none"
            placeholder='{"data":"Hello"}'
          />
          <Text style={styles.hint}>Structure must match the message type fields.</Text>

          <View style={styles.buttonRow}>
            <TouchableOpacity
              style={[styles.button, advertised && styles.buttonSecondary]}
              onPress={advertised ? handleUnadvertise : handleAdvertise}
            >
              <Text style={styles.buttonText}>{advertised ? 'UNADVERTISE' : 'ADVERTISE'}</Text>
            </TouchableOpacity>

            <TouchableOpacity
                style={[styles.button, (!advertised || !isConnected) && styles.buttonDisabled]}
              onPress={handlePublish}
                disabled={!advertised || !isConnected}
            >
              <Text style={styles.buttonText}>PUBLISH</Text>
            </TouchableOpacity>
          </View>

          {message ? (
            <View style={[styles.messageBanner, messageType === 'error' && styles.messageBannerError, messageType === 'success' && styles.messageBannerSuccess]}>
              <Text style={styles.messageBannerText}>{message}</Text>
            </View>
          ) : null}

          {!isConnected && (
            <View style={styles.warning}>
              <Text style={styles.warningText}>Not connected to rosbridge.</Text>
            </View>
          )}
        </View>
      </ScrollView>

    </View>
  );
}

const styles = StyleSheet.create({
  container: { flex: 1, backgroundColor: theme.background.primary },
  header: {
    backgroundColor: theme.background.secondary,
    paddingHorizontal: 16,
    paddingVertical: 12,
    paddingTop: 40,
    borderBottomWidth: 2,
    borderBottomColor: theme.border.primary,
  },
  title: { fontSize: 18, fontWeight: '700', color: theme.text.primary, marginBottom: 4 },
  subtitle: { fontSize: 12, color: theme.text.muted },
  scroll: { flex: 1 },
  scrollContent: { paddingBottom: 24 },
  form: { padding: 20 },
  label: { fontSize: 11, fontWeight: '600', color: theme.text.accent, letterSpacing: 1, marginBottom: 6 },
  input: {
    backgroundColor: theme.background.input,
    borderWidth: 1,
    borderColor: theme.border.primary,
    borderRadius: 8,
    padding: 12,
    fontSize: 14,
    color: theme.text.primary,
    marginBottom: 14,
  },
  multiline: { minHeight: 100, textAlignVertical: 'top' },
  hint: { fontSize: 11, color: theme.text.muted, marginTop: -8, marginBottom: 12 },
  buttonRow: { flexDirection: 'row', justifyContent: 'space-between', marginTop: 8 },
  button: {
    flex: 1,
    backgroundColor: theme.accent.primary,
    padding: 14,
    borderRadius: 8,
    alignItems: 'center',
    marginRight: 8,
  },
  buttonSecondary: {
    backgroundColor: theme.accent.error,
  },
  buttonDisabled: { opacity: 0.4 },
  buttonText: { color: '#fff', fontSize: 12, fontWeight: '700', letterSpacing: 1 },
  warning: {
    marginTop: 12,
    padding: 10,
    borderRadius: 6,
    backgroundColor: theme.background.card,
    borderWidth: 1,
    borderColor: theme.accent.warning,
  },
  warningText: { color: theme.accent.warning, fontSize: 12, textAlign: 'center' },
  messageBanner: {
    marginTop: 12,
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
    fontWeight: '600' 
  },
});