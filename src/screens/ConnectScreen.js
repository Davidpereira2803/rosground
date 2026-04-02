import React, { useState } from 'react';
import { View, Text, TextInput, TouchableOpacity, StyleSheet, ScrollView, useWindowDimensions, Linking } from 'react-native';
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

  const { width, height } = useWindowDimensions();
  const isLandscape = width > height;
  const sideGutter = isLandscape ? Math.max(24, Math.round(width * 0.08)) : 20;

  const isDev = __DEV__ || Constants.appOwnership === 'expo';

  const appVersion = Constants.expoConfig?.version ?? '0.0.0';
  
  const handleConnect = async () => {
    setError('');
    setConnecting(true);

    const trimmedIp = ip.trim();
    const trimmedRosbridgePort = rosbridgePort.trim();
    const trimmedVideoPort = videoPort.trim();

    const ipPattern = /^(\d{1,3}\.){3}\d{1,3}$/;
    const portPattern = /^\d+$/;

    const ipIsValid = ipPattern.test(trimmedIp) && trimmedIp.split('.').every(part => Number(part) >= 0 && Number(part) <= 255);
    const rosbridgePortNumber = Number(trimmedRosbridgePort);
    const videoPortNumber = Number(trimmedVideoPort);

    if (!ipIsValid) {
      setError('Enter a valid IPv4 address, such as 192.168.1.42.');
      setConnecting(false);
      return;
    }

    if (!portPattern.test(trimmedRosbridgePort) || rosbridgePortNumber < 1 || rosbridgePortNumber > 65535) {
      setError('Enter a valid rosbridge port between 1 and 65535.');
      setConnecting(false);
      return;
    }

    if (trimmedVideoPort && (!portPattern.test(trimmedVideoPort) || videoPortNumber < 1 || videoPortNumber > 65535)) {
      setError('Enter a valid video port between 1 and 65535, or leave it blank.');
      setConnecting(false);
      return;
    }

    try {
      await connectToROS(trimmedIp, trimmedRosbridgePort, trimmedVideoPort);
      navigation.navigate('Dashboard');
    } catch (err) {
      setError(err?.message === 'Connection timed out'
        ? 'Could not reach rosbridge in time. Check the IP, port, and network.'
        : 'Connection failed. Please check the IP, port, and network.');
      console.error(err);
    } finally {
      setConnecting(false);
    }
  };

  const handleDevSkip = () => {
    setError('Dev mode preview: dashboard opens without a live ROS connection.');
    console.log('DEV MODE: Skipping connection validation');
    navigation.navigate('Dashboard');
  };

  return (
    <ScrollView style={styles.container} contentContainerStyle={styles.contentContainer}>
      <View style={[styles.header, { paddingHorizontal: sideGutter }]}>
        <Text style={styles.title}>ROS Monitor</Text>
        <Text style={styles.subtitle}>Connect to ROS Bridge</Text>
        {isDev && (
          <View style={styles.devBadge}>
            <Text style={styles.devBadgeText}>DEV MODE</Text>
          </View>
        )}
      </View>

      <View style={[styles.formContainer, { paddingHorizontal: sideGutter }]}>
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

        {false && (
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
        )}

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

        {error ? (
          <View style={styles.messageBanner}>
            <Text style={styles.messageBannerText}>{error}</Text>
          </View>
        ) : null}
      </View>

      <View style={[styles.footer, { paddingHorizontal: sideGutter }]}>
        <TouchableOpacity
          onPress={() => navigation.navigate('HowToUse')}
          style={styles.helpLink}
        >
          <Text style={styles.helpLinkText}>Need help? View Setup Guide →</Text>
        </TouchableOpacity>

        {/* Website Link */}
        <TouchableOpacity
          onPress={() => Linking.openURL('https://pearlabs.dev/rosground')}
          style={styles.websiteLink}
        >
          <Text style={styles.websiteLinkText}>Visit Website →</Text>
        </TouchableOpacity>

        <Text style={styles.version}>Version {appVersion}</Text>
        <Text style={styles.disclaimer}>
          Ensure your device and robot are on the same network.{'\n'}
          This app requires rosbridge_server running on your robot.
        </Text>
      </View>

      {/*<AdBanner />*/}
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
  messageBanner: {
    marginTop: 16,
    backgroundColor: theme.background.card,
    borderColor: theme.accent.error,
    borderWidth: 1,
    borderRadius: 10,
    padding: 12,
  },
  messageBannerText: {
    color: theme.accent.error,
    textAlign: 'center',
    fontSize: 13,
    lineHeight: 18,
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
  helpLink: {
    marginTop: 16,
    paddingVertical: 8,
  },
  helpLinkText: {
    fontSize: 13,
    color: theme.accent.primary,
    textAlign: 'center',
    fontWeight: '600',
  },
  websiteLink: {
    marginTop: 12,
    paddingVertical: 8,
  },
  websiteLinkText: {
    fontSize: 13,
    color: theme.accent.primary,
    textAlign: 'center',
    fontWeight: '600',
  },
});