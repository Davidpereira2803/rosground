import React from "react";
import { View, StyleSheet, Platform } from "react-native";
import Constants from "expo-constants";
import { BannerAd, BannerAdSize, TestIds } from "react-native-google-mobile-ads";
import { useAdConsentContext } from "../context/AdConsentContext";


{/*
const prodUnitId = 
  Platform.OS === "ios"
    ? ADS_CONFIG.IOS_BANNER_UNIT_ID
    : ADS_CONFIG.ANDROID_BANNER_UNIT_ID;
*/}

const adUnitId = __DEV__
  ? TestIds.BANNER
  : ("" || "");
  
export const AdBanner = () => {
  const { isReady, canShowAds, canShowPersonalizedAds } = useAdConsentContext();

  if (!isReady || !canShowAds || !adUnitId) {
    return <View style={styles.placeholder} />;
  }

  return (
    <BannerAd
      unitId={adUnitId}
      size={BannerAdSize.ANCHORED_ADAPTIVE_BANNER}
      requestOptions={{
        requestNonPersonalizedAdsOnly: !canShowPersonalizedAds,
      }}
    />
  );
};

const styles = StyleSheet.create({
  placeholder: { height: 50 },
});