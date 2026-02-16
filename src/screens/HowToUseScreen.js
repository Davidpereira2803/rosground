import React from 'react';
import { View, Text, ScrollView, TouchableOpacity, StyleSheet } from 'react-native';
import { theme } from '../theme/colors';

const checks = [
  '1) rosbridge websocket is running on your ROS machine.',
  '2) Phone and robot are on the same Wi-Fi or LAN.',
];

const steps = [
  {
    number: '1',
    title: 'Connect to ROS',
    body: 'Enter the robot IP, rosbridge port (default 9090), and video port if used. Tap CONNECT.',
  },
  {
    number: '2',
    title: 'Add a topic',
    body: 'Tap ADD TOPIC, pick from the list or enter name and type, then SUBSCRIBE.',
  },
  {
    number: '3',
    title: 'Monitor data',
    body: 'Dashboard cards show the latest message. Tap X to unsubscribe.',
  },
  {
    number: '4',
    title: 'Publish test',
    body: 'Open PUBLISH TEST to advertise a topic and send JSON messages.',
  },
];

const tips = [
  '1) If available topics are empty, make sure rosapi is running.',
  '2) Topic type must match exactly (case-sensitive).',
  '3) Connection issues usually mean IP, ports, or firewall.',
];

export default function HowToUseScreen({ navigation }) {
  return (
    <View style={styles.container}>
      <View style={styles.header}>
        <TouchableOpacity
          onPress={() => navigation.goBack()}
          style={styles.backButton}
          accessibilityRole="button"
          accessibilityLabel="Go Back"
        >
          <Text style={styles.backText}>Back</Text>
        </TouchableOpacity>
        <Text style={styles.title}>How to Use</Text>
        <Text style={styles.subtitle}>Quick guide to connect, subscribe, and publish.</Text>
      </View>

      <ScrollView style={styles.scroll} contentContainerStyle={styles.content}>
        <View style={styles.section}>
          <Text style={styles.sectionTitle}>Before you start</Text>
          <View style={styles.card}>
            <Text style={styles.cardText}>{checks.join('\n')}</Text>
          </View>
        </View>

        <View style={styles.section}>
          <Text style={styles.sectionTitle}>Quick steps</Text>
          {steps.map((step) => (
            <View key={step.number} style={styles.stepCard}>
              <View style={styles.stepNumber}>
                <Text style={styles.stepNumberText}>{step.number}</Text>
              </View>
              <View style={styles.stepBody}>
                <Text style={styles.stepTitle}>{step.title}</Text>
                <Text style={styles.stepText}>{step.body}</Text>
              </View>
            </View>
          ))}
        </View>

        <View style={styles.section}>
          <Text style={styles.sectionTitle}>Troubleshooting</Text>
          <View style={styles.card}>
            <Text style={styles.cardText}>{tips.join('\n')}</Text>
          </View>
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
    paddingTop: 40,
    paddingBottom: 14,
    borderBottomWidth: 2,
    borderBottomColor: theme.border.primary,
  },
  backButton: {
    alignSelf: 'flex-start',
    paddingVertical: 6,
    paddingHorizontal: 10,
    borderRadius: 8,
    backgroundColor: theme.background.card,
    borderWidth: 1,
    borderColor: theme.border.subtle,
    marginBottom: 10,
  },
  backText: {
    color: theme.text.primary,
    fontWeight: '600',
    fontSize: 12,
  },
  title: {
    fontSize: 20,
    fontWeight: '700',
    color: theme.text.primary,
    marginBottom: 6,
  },
  subtitle: {
    color: theme.text.secondary,
    fontSize: 13,
  },
  scroll: {
    flex: 1,
  },
  content: {
    padding: 16,
    paddingBottom: 30,
  },
  section: {
    marginBottom: 18,
  },
  sectionTitle: {
    fontSize: 13,
    fontWeight: '700',
    color: theme.text.accent,
    letterSpacing: 1,
    marginBottom: 12,
  },
  card: {
    backgroundColor: theme.background.card,
    padding: 14,
    borderRadius: 12,
    borderWidth: 1,
    borderColor: theme.border.primary,
  },
  cardText: {
    color: theme.text.secondary,
    fontSize: 13,
    lineHeight: 18,
  },
  stepCard: {
    flexDirection: 'row',
    backgroundColor: theme.background.card,
    padding: 14,
    borderRadius: 12,
    borderWidth: 1,
    borderColor: theme.border.primary,
    marginBottom: 12,
  },
  stepNumber: {
    width: 26,
    height: 26,
    borderRadius: 13,
    backgroundColor: theme.background.input,
    alignItems: 'center',
    justifyContent: 'center',
    borderWidth: 1,
    borderColor: theme.border.secondary,
    marginRight: 12,
  },
  stepNumberText: {
    color: theme.text.accent,
    fontWeight: '700',
    fontSize: 12,
  },
  stepBody: {
    flex: 1,
  },
  stepTitle: {
    color: theme.text.primary,
    fontWeight: '700',
    fontSize: 14,
    marginBottom: 4,
  },
  stepText: {
    color: theme.text.secondary,
    fontSize: 13,
    lineHeight: 18,
  },
});