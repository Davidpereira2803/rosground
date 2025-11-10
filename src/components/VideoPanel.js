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

  const hasStream = isConnected && streamUrl;

  if (!hasStream) {
    return (
      <View style={styles.containerCompact}>
        <View style={styles.headerRow}>
          <Text style={styles.title}>LIVE VIDEO</Text>
          <View style={styles.disconnectedBadge}>
            <Text style={styles.disconnectedBadgeText}>Not Available</Text>
          </View>
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
  containerCompact: {
    backgroundColor: theme.background.card,
    borderColor: theme.border.primary,
    borderWidth: 1,
    borderRadius: 12,
    padding: 12,
    marginBottom: 16,
  },
  headerRow: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
  },
  title: {
    fontSize: 12,
    fontWeight: '700',
    color: theme.text.accent,
    letterSpacing: 1,
  },
  disconnectedBadge: {
    backgroundColor: theme.background.primary,
    paddingHorizontal: 8,
    paddingVertical: 4,
    borderRadius: 6,
    borderWidth: 1,
    borderColor: theme.border.subtle,
  },
  disconnectedBadgeText: {
    color: theme.text.muted,
    fontSize: 10,
    fontWeight: '600',
  },
  videoContainer: {
    borderRadius: 8,
    overflow: 'hidden',
    borderWidth: 1,
    borderColor: theme.border.subtle,
    marginTop: 12,
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