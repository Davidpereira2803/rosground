import React, { useMemo } from 'react';
import { View, Text, Image, StyleSheet } from 'react-native';
import { useROS } from '../context/ROSContext';
import { theme } from '../theme/colors';

export default function VideoPanel() {
  const { connectionInfo, isConnected } = useROS();

  const streamUrl = useMemo(() => {
    if (connectionInfo.ip && connectionInfo.videoPort) {
      return `http://${connectionInfo.ip}:${connectionInfo.videoPort}/stream`;
    }
    return null;
  }, [connectionInfo.ip, connectionInfo.videoPort]);

  if (!isConnected || !streamUrl) {
    return (
      <View style={styles.container}>
        <Text style={styles.title}>LIVE VIDEO</Text>
        <View style={styles.notConnectedContainer}>
          <Text style={styles.notConnectedText}>Video - Not connected</Text>
        </View>
      </View>
    );
  }

  return (
    <View style={styles.container}>
      <Text style={styles.title}>LIVE VIDEO</Text>
      <View style={styles.videoContainer}>
        <Image
          source={{ uri: streamUrl }}
          style={styles.videoImage}
          resizeMode="contain"
        />
      </View>
      <Text style={styles.debugUrl}>{streamUrl}</Text>
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    backgroundColor: theme.background.card,
    borderColor: theme.border.primary,
    borderWidth: 1,
    borderRadius: 12,
    padding: 16,
    marginBottom: 20,
  },
  title: {
    fontSize: 12,
    fontWeight: '700',
    color: theme.text.accent,
    marginBottom: 12,
    letterSpacing: 1,
  },
  notConnectedContainer: {
    height: 200,
    backgroundColor: theme.background.primary,
    justifyContent: 'center',
    alignItems: 'center',
    borderRadius: 8,
    borderWidth: 1,
    borderColor: theme.border.subtle,
  },
  notConnectedText: {
    color: theme.text.muted,
    fontSize: 14,
  },
  videoContainer: {
    borderRadius: 8,
    overflow: 'hidden',
    borderWidth: 1,
    borderColor: theme.border.subtle,
  },
  videoImage: {
    width: '100%',
    height: 200,
    backgroundColor: theme.background.primary,
  },
  debugUrl: {
    fontSize: 10,
    color: theme.text.muted,
    marginTop: 8,
  },
});