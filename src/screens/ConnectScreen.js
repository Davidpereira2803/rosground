import React, { useState } from 'react';
import { View, Text, TextInput, TouchableOpacity, StyleSheet, ScrollView } from 'react-native';
import { useROS } from '../context/ROSContext';
import { theme } from '../theme/colors';
import Constants from 'expo-constants';

export default function ConnectScreen({ navigation }) {
  const { connectToROS } = useROS();
  const [ip, setIp] = useState('192.168.178.31');
  const [rosbridgePort, setRosbridgePort] = useState('9090');
  const [videoPort, setVideoPort] = useState('8080');
  const [error, setError] = useState('');
  const [connecting, setConnecting] = useState(false);

  const isDev = __DEV__ || Constants.appOwnership === 'expo';

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
      setError('Connection failed. Please check the IP and Port.');
      console.error(err);
    } finally {
      setConnecting(false);
    }
  };

  const handleDevSkip = () => {
    console.log('DEV MODE: Skipping connection validation');
    navigation.navigate('Dashboard');
  };

  return (
    <ScrollView style={styles.container} contentContainerStyle={styles.contentContainer}>
      <View style={styles.header}>
        <Text style={styles.title}>ROS Monitor</Text>
        <Text style={styles.subtitle}>Connect to ROS Bridge</Text>
        {isDev && (
          <View style={styles.devBadge}>
            <Text style={styles.devBadgeText}>DEV MODE</Text>
          </View>
        )}
      </View>

      <View style={styles.formContainer}>
        <View style={styles.inputContainer}>
          <Text style={styles.label}>ROBOT IP / COMPUTER IP</Text>
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

        {/* Development mode skip button */}
        {isDev && (
          <TouchableOpacity
            style={styles.devButton}
            onPress={handleDevSkip}
          >
            <Text style={styles.devButtonText}>SKIP (DEV ONLY)</Text>
          </TouchableOpacity>
        )}

        {error ? <Text style={styles.errorText}>{error}</Text> : null}
      </View>

      <View style={styles.footer}>
        <Text style={styles.version}>Version 1.0.0</Text>
        <Text style={styles.disclaimer}>
          Ensure your device and robot are on the same network.{'\n'}
          This app requires rosbridge_server running on your robot.
        </Text>
      </View>
    </ScrollView>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: theme.background.primary,
  },
  contentContainer: {
    flexGrow: 1,
    justifyContent: 'space-between',
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
  devBadge: {
    marginTop: 10,
    backgroundColor: theme.accent.warning,
    paddingHorizontal: 12,
    paddingVertical: 4,
    borderRadius: 4,
  },
  devBadgeText: {
    color: '#000',
    fontSize: 10,
    fontWeight: '700',
    letterSpacing: 1,
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
  devButton: {
    backgroundColor: theme.accent.warning,
    borderRadius: 8,
    padding: 12,
    alignItems: 'center',
    marginTop: 12,
  },
  devButtonText: {
    color: '#000',
    fontSize: 14,
    fontWeight: '700',
    letterSpacing: 1,
  },
  errorText: {
    color: theme.accent.error,
    marginTop: 16,
    textAlign: 'center',
    fontSize: 14,
  },
  footer: {
    paddingHorizontal: 20,
    paddingBottom: 30,
    paddingTop: 20,
    alignItems: 'center',
  },
  version: {
    fontSize: 12,
    color: theme.text.muted,
    marginBottom: 10,
    fontWeight: '600',
  },
  disclaimer: {
    fontSize: 11,
    color: theme.text.muted,
    textAlign: 'center',
    lineHeight: 16,
    opacity: 0.7,
  },
});