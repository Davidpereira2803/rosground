import React from 'react';
import { View, Text, ScrollView, TouchableOpacity, StyleSheet } from 'react-native';
import MaterialCommunityIcons from '@expo/vector-icons/MaterialCommunityIcons';
import { theme } from '../theme/colors';

const checks = [
  'Robot/PC has ROS installed (ROS 2 or ROS 1).',
  'Phone and robot are on the same Wi-Fi/LAN.',
  'Firewall allows TCP 9090 (rosbridge) and 8080 (optional video).',
];

const setupCommands = [
  {
    title: '1) Install rosbridge (robot/PC)',
    lines: [
      'sudo apt update',
      'sudo apt install ros-${ROS_DISTRO}-rosbridge-server',
    ],
  },
  {
    title: '2) Source ROS environment (every new terminal)',
    lines: ['source /opt/ros/<your-ros-distro>/setup.bash'],
  },
  {
    title: '3A) Start rosbridge (ROS 2)',
    lines: ['ros2 launch rosbridge_server rosbridge_websocket_launch.xml'],
  },
  {
    title: '3B) Start rosbridge (ROS 1)',
    lines: ['roslaunch rosbridge_server rosbridge_websocket.launch'],
  },
  {
    title: '4) Verify ROS + network',
    lines: [
      'ros2 topic list    # or: rostopic list',
      'ip a               # check robot IP',
      'ss -ltnp | grep 9090',
    ],
  },
];

const steps = [
  {
    number: '1',
    title: 'Find robot IP',
    body: 'On robot/PC run "ip a", then use that IP in the app (example: 192.168.1.100).',
    icon: 'ip-network',
  },
  {
    number: '2',
    title: 'Connect in app',
    body: 'Set Robot IP + Rosbridge Port 9090. Tap CONNECT.',
    icon: 'wifi-check',
  },
  {
    number: '3',
    title: 'Add and subscribe topic',
    body: 'Tap ADD TOPIC, choose from list (or enter manually), then SUBSCRIBE.',
    icon: 'plus-circle',
  },
  {
    number: '4',
    title: 'Publish test',
    body: 'Open PUBLISH TEST, advertise topic first, then send valid JSON.',
    icon: 'send',
  },
];

const tips = [
  { icon: 'robot', text: 'If topic list is empty, rosbridge/rosapi is likely not running.' },
  { icon: 'alphabetical', text: 'Topic type must match exactly (case-sensitive).' },
  { icon: 'wifi-off', text: 'Most connection failures are wrong IP, closed port, or firewall.' },
];

export default function HowToUseScreen({ navigation }) {
  return (
    <View style={styles.container}>
      <View style={styles.header}>
        <View style={styles.headerTextContainer}>
          <Text style={styles.headerTitle}>How to Use</Text>
          <Text style={styles.headerSubtitle}>Quick guide to connect, subscribe, and publish</Text>
        </View>
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

        {/* Robot/PC setup commands */}
        <View style={styles.section}>
          <View style={styles.sectionHeaderContainer}>
            <MaterialCommunityIcons name="console" size={18} color={theme.text.accent} />
            <Text style={styles.sectionTitle}>Robot/PC setup commands</Text>
          </View>
          <View style={styles.card}>
            {setupCommands.map((block, idx) => (
              <View key={idx} style={styles.commandBlock}>
                <Text style={styles.commandTitle}>{block.title}</Text>
                {block.lines.map((line, lineIdx) => (
                  <Text key={lineIdx} style={styles.commandLine}>{line}</Text>
                ))}
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
    paddingVertical: 16,
    paddingTop: 40,
    borderBottomWidth: 2,
    borderBottomColor: theme.border.primary,
    flexDirection: 'row',
    alignItems: 'center',
  },
  backButton: {
    paddingRight: 12,
    marginRight: 8,
  },
  backText: {
    fontSize: 20,
    fontWeight: '700',
    color: theme.text.primary,
  },
  headerTextContainer: {
    flex: 1,
  },
  headerTitle: {
    fontSize: 18,
    fontWeight: '700',
    color: theme.text.primary,
    marginBottom: 2,
  },
  headerSubtitle: {
    fontSize: 12,
    color: theme.text.secondary,
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
  commandBlock: {
    marginBottom: 12,
  },
  commandTitle: {
    color: theme.text.primary,
    fontSize: 13,
    fontWeight: '700',
    marginBottom: 6,
  },
  commandLine: {
    color: theme.text.secondary,
    fontSize: 12,
    lineHeight: 18,
    fontFamily: 'monospace',
    backgroundColor: theme.background.input,
    borderWidth: 1,
    borderColor: theme.border.secondary,
    borderRadius: 8,
    paddingHorizontal: 8,
    paddingVertical: 6,
    marginBottom: 6,
  },
});