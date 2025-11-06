import React from 'react';
import { NavigationContainer, DefaultTheme } from '@react-navigation/native';
import { createNativeStackNavigator } from '@react-navigation/native-stack';
import { StatusBar } from 'expo-status-bar';
import { ROSProvider } from './src/context/ROSContext';
import ConnectScreen from './src/screens/ConnectScreen';
import DashboardScreen from './src/screens/DashboardScreen';
import TopicBrowserScreen from './src/screens/TopicBrowserScreen';
import { theme } from './src/theme/colors';

const Stack = createNativeStackNavigator();

const MyTheme = {
  ...DefaultTheme,
  colors: {
    ...DefaultTheme.colors,
    primary: theme.accent.primary,
    background: theme.background.primary,
    card: theme.background.secondary,
    text: theme.text.primary,
    border: theme.border.primary,
  },
};

export default function App() {
  return (
    <ROSProvider>
      <StatusBar style="light" />
      <NavigationContainer theme={MyTheme}>
        <Stack.Navigator 
          initialRouteName="Connect"
          screenOptions={{
            headerStyle: {
              backgroundColor: theme.background.secondary,
            },
            headerTintColor: theme.text.primary,
          }}
        >
          <Stack.Screen 
            name="Connect" 
            component={ConnectScreen}
            options={{ 
              headerShown: false,
            }}
          />
          <Stack.Screen 
            name="Dashboard" 
            component={DashboardScreen}
            options={{
              headerShown: false,
            }}
          />
          <Stack.Screen 
            name="TopicBrowser" 
            component={TopicBrowserScreen}
            options={{ 
              title: 'Add Topic',
              headerShown: false,
            }}
          />
        </Stack.Navigator>
      </NavigationContainer>
    </ROSProvider>
  );
}
