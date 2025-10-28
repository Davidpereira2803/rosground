import React, { useState } from 'react';
import { View, Text, TextInput, TouchableOpacity, StyleSheet } from 'react-native';
import { useROS } from '../context/ROSContext';
import { theme } from '../theme/colors';

export default function ConnectScreen({ navigation }) {
  const { connectToROS } = useROS();
  const [ip, setIp] = useState('192.168.178.31');
  const [rosbridgePort, setRosbridgePort] = useState('9090');
  const [videoPort, setVideoPort] = useState('8080');
  const [error, setError] = useState('');
  const [connecting, setConnecting] = useState(false);

  const handleConnect = async () => {
    setError('');
    setConnecting(true);

    if (!ip.trim()) {
      setError('Please enter an IP address');
      setConnecting(false);
      return;
    }

    if (!rosbridgePort.trim()) {
      setError('Please enter a rosbridge port');
      setConnecting(false);
      return;
    }

    try {
      await connectToROS(ip, rosbridgePort, videoPort);
      navigation.navigate('Dashboard');
    } catch (err) {
      setError('Connection failed. Please check the IP and port.');
      console.error(err);
    } finally {
      setConnecting(false);
    }
  };

  return (
    <View style={styles.container}>
      <View style={styles.header}>
        <Text style={styles.title}>ROSground</Text>
        <Text style={styles.subtitle}>Connect to ROS Bridge</Text>
      </View>

      <View style={styles.formContainer}>
        <View style={styles.inputContainer}>
          <Text style={styles.label}>ROBOT IP</Text>
          <TextInput
            style={styles.input}
            value={ip}
            onChangeText={setIp}
            placeholder="e.g. 192.168.1.42"
            placeholderTextColor={theme.text.placeholder}
            keyboardType="default"
            autoCapitalize="none"
            autoCorrect={false}
          />
        </View>

        <View style={styles.inputContainer}>
          <Text style={styles.label}>ROSBRIDGE PORT</Text>
          <TextInput
            style={styles.input}
            value={rosbridgePort}
            onChangeText={setRosbridgePort}
            placeholder="9090"
            placeholderTextColor={theme.text.placeholder}
            keyboardType="numeric"
          />
        </View>

        <View style={styles.inputContainer}>
          <Text style={styles.label}>VIDEO PORT</Text>
          <TextInput
            style={styles.input}
            value={videoPort}
            onChangeText={setVideoPort}
            placeholder="8080"
            placeholderTextColor={theme.text.placeholder}
            keyboardType="numeric"
          />
        </View>

        <TouchableOpacity
          style={[styles.button, connecting && styles.buttonDisabled]}
          onPress={handleConnect}
          disabled={connecting}
        >
          <Text style={styles.buttonText}>
            {connecting ? "CONNECTING..." : "CONNECT"}
          </Text>
        </TouchableOpacity>

        {error ? <Text style={styles.errorText}>{error}</Text> : null}
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
    paddingTop: 60,
    paddingBottom: 40,
    paddingHorizontal: 20,
    alignItems: 'center',
  },
  title: {
    fontSize: 36,
    fontWeight: '700',
    color: theme.accent.primary,
    marginBottom: 8,
  },
  subtitle: {
    fontSize: 16,
    color: theme.text.secondary,
  },
  formContainer: {
    paddingHorizontal: 20,
  },
  inputContainer: {
    marginBottom: 24,
  },
  label: {
    fontSize: 12,
    marginBottom: 8,
    fontWeight: '600',
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
    opacity: 0.6,
  },
  buttonText: {
    color: '#FFFFFF',
    fontSize: 16,
    fontWeight: '700',
    letterSpacing: 1,
  },
  errorText: {
    color: theme.accent.error,
    marginTop: 16,
    textAlign: 'center',
    fontSize: 14,
  },
});