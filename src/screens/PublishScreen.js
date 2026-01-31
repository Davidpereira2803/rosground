import React, { useState } from 'react';
import { View, Text, TextInput, TouchableOpacity, StyleSheet, Alert, ScrollView } from 'react-native';
import { useROS } from '../context/ROSContext';
import { theme } from '../theme/colors';

export default function PublishScreen({ navigation }) {
  const { isConnected, advertiseTopic, publishMessage, unadvertiseTopic } = useROS();
  const [topicName, setTopicName] = useState('/chatter');
  const [topicType, setTopicType] = useState('std_msgs/msg/String');
  const [rawMessage, setRawMessage] = useState('{"data":"Hello from phone"}');
  const [advertised, setAdvertised] = useState(false);

  const handleAdvertise = () => {
    if (!isConnected) {
      Alert.alert('Not Connected', 'Connect first.');
      return;
    }
    if (!topicName.trim() || !topicType.trim()) {
      Alert.alert('Error', 'Topic name and type required.');
      return;
    }
    advertiseTopic(topicName.trim(), topicType.trim());
    setAdvertised(true);
  };

  const handlePublish = () => {
    if (!advertised) {
      Alert.alert('Not Advertised', 'Advertise the topic first.');
      return;
    }
    try {
      const msgObj = JSON.parse(rawMessage);
      publishMessage(topicName.trim(), msgObj);
      Alert.alert('Published', 'Message sent.');
    } catch (e) {
      Alert.alert('Invalid JSON', e.message);
    }
  };

  const handleUnadvertise = () => {
    if (advertised) {
      unadvertiseTopic(topicName.trim());
      setAdvertised(false);
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
              style={[styles.button, !advertised && styles.buttonDisabled]}
              onPress={handlePublish}
              disabled={!advertised}
            >
              <Text style={styles.buttonText}>PUBLISH</Text>
            </TouchableOpacity>
          </View>

          {!isConnected && (
            <View style={styles.warning}>
              <Text style={styles.warningText}>Not connected to rosbridge.</Text>
            </View>
          )}
        </View>
      </ScrollView>

      <View style={styles.footer}>
        <TouchableOpacity onPress={() => navigation.goBack()}>
          <Text style={styles.footerText}>Close</Text>
        </TouchableOpacity>
      </View>
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