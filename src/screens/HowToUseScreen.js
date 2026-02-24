import React from 'react';
import { View, Text, ScrollView, TouchableOpacity, StyleSheet } from 'react-native';
import MaterialCommunityIcons from '@expo/vector-icons/MaterialCommunityIcons';
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
    icon: 'wifi-check',
  },
  {
    number: '2',
    title: 'Add a topic',
    body: 'Tap ADD TOPIC, pick from the list or enter name and type, then SUBSCRIBE.',
    icon: 'plus-circle',
  },
  {
    number: '3',
    title: 'Monitor data',
    body: 'Dashboard cards show the latest message. Tap X to unsubscribe.',
    icon: 'monitor-dashboard',
  },
  {
    number: '4',
    title: 'Publish test',
    body: 'Open PUBLISH TEST to advertise a topic and send JSON messages.',
    icon: 'send',
  },
];

const tips = [
  { icon: 'robot', text: '1) If available topics are empty, make sure rosapi is running.' },
  { icon: 'alphabetical', text: '2) Topic type must match exactly (case-sensitive).' },
  { icon: 'wifi-off', text: '3) Connection issues usually mean IP, ports, or firewall.' },
];

export default function HowToUseScreen({ navigation }) {
  return (
    <View style={styles.container}>
      <View style={styles.header}>
        <Text style={styles.title}>How to Use</Text>
        <Text style={styles.subtitle}>Quick guide to connect, subscribe, and publish.</Text>
      </View>

      <ScrollView style={styles.scroll} contentContainerStyle={styles.content}>
        {/* Before you start section */}
        <View style={styles.section}>
          <View style={styles.sectionHeaderContainer}>
            <MaterialCommunityIcons name="lightbulb-on" size={18} color={theme.text.accent} />
            <Text style={styles.sectionTitle}>Before you start</Text>
          </View>
          <View style={styles.card}>
            {checks.map((check, idx) => (
              <View key={idx} style={styles.checkItem}>
                <MaterialCommunityIcons name="check-circle-outline" size={16} color={theme.text.accent} />
                <Text style={styles.cardText}>{check}</Text>
              </View>
            ))}
          </View>
        </View>

        {/* Quick steps section */}
        <View style={styles.section}>
          <View style={styles.sectionHeaderContainer}>
            <MaterialCommunityIcons name="format-list-numbered" size={18} color={theme.text.accent} />
            <Text style={styles.sectionTitle}>Quick steps</Text>
          </View>
          {steps.map((step) => (
            <View key={step.number} style={styles.stepCard}>
              <View style={styles.stepIconContainer}>
                <MaterialCommunityIcons name={step.icon} size={24} color={theme.text.accent} />
              </View>
              <View style={styles.stepBody}>
                <Text style={styles.stepTitle}>{step.title}</Text>
                <Text style={styles.stepText}>{step.body}</Text>
              </View>
            </View>
          ))}
        </View>

        {/* Troubleshooting section */}
        <View style={styles.section}>
          <View style={styles.sectionHeaderContainer}>
            <MaterialCommunityIcons name="wrench" size={18} color={theme.text.accent} />
            <Text style={styles.sectionTitle}>Troubleshooting</Text>
          </View>
          <View style={styles.card}>
            {tips.map((tip, idx) => (
              <View key={idx} style={styles.tipItem}>
                <MaterialCommunityIcons name={tip.icon} size={16} color={theme.text.secondary} />
                <Text style={styles.cardText}>{tip.text}</Text>
              </View>
            ))}
          </View>
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
  container: {
    flex: 1,
    backgroundColor: theme.background.primary,
  },
  header: {
    backgroundColor: theme.background.secondary,
    paddingHorizontal: 16,
    paddingTop: 40,
    paddingBottom: 16,
    borderBottomWidth: 2,
    borderBottomColor: theme.border.primary,
  },
  title: {
    fontSize: 20,
    fontWeight: '700',
    color: theme.text.primary,
    marginBottom: 4,
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
    paddingBottom: 40,
  },
  section: {
    marginBottom: 20,
  },
  sectionHeaderContainer: {
    flexDirection: 'row',
    alignItems: 'center',
    marginBottom: 12,
  },
  sectionTitle: {
    fontSize: 13,
    fontWeight: '700',
    color: theme.text.accent,
    letterSpacing: 1,
    marginLeft: 8,
  },
  card: {
    backgroundColor: theme.background.card,
    padding: 14,
    borderRadius: 12,
    borderWidth: 1,
    borderColor: theme.border.primary,
  },
  checkItem: {
    flexDirection: 'row',
    alignItems: 'flex-start',
    marginBottom: 10,
  },
  tipItem: {
    flexDirection: 'row',
    alignItems: 'flex-start',
    marginBottom: 10,
  },
  cardText: {
    color: theme.text.secondary,
    fontSize: 13,
    lineHeight: 18,
    marginLeft: 10,
    flex: 1,
  },
  stepCard: {
    flexDirection: 'row',
    backgroundColor: theme.background.card,
    padding: 14,
    borderRadius: 12,
    borderWidth: 1,
    borderColor: theme.border.primary,
    marginBottom: 12,
    alignItems: 'flex-start',
  },
  stepIconContainer: {
    width: 40,
    height: 40,
    borderRadius: 10,
    backgroundColor: theme.background.input,
    alignItems: 'center',
    justifyContent: 'center',
    borderWidth: 1,
    borderColor: theme.border.secondary,
    marginRight: 12,
    flexShrink: 0,
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