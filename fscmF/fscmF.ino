#include "fscmF.h"
#include "fscmFFunctions.h"
void setup() {
  Serial.begin(2000000);
  pinMode(13, OUTPUT);
  pinMode(12, INPUT_PULLUP);
  fscmFFSetupSensors();
  fscmFFSetupCoproc();
  fscmFFSetupRadio();
}
void loop() {
  fscmFFReadSensors();
  fscmFFRadio();
  if (gotFscmTMsgLast) {
    fscmFFCoproc();
    fscmFFWaypoints();
  }
  fscmFFHomeSet();
  fscmFGAlt = fscmFFAltCalc();
  if (millis() - fscmFLastRecvMillis < SHUTOFF_AFTER_MILLIS) {//connected
    digitalWrite(13, fscmFEnabled);
  } else {//signal loss
    if (millis() % 500 < 200) {
      digitalWrite(13, HIGH);
    } else {
      digitalWrite(13, LOW);
    }
  }
}
void fscmFFDataToParseFromFscmC() {
  fscmCPitch = fscmFFParseDataFscmCFl();
  fscmCRoll = fscmFFParseDataFscmCFl();
}
void fscmFFDataToSendToFscmC() {
  fscmFFSendDataFscmCBl(fscmFEnabled);
  fscmFFSendDataFscmCBl(fscmHomeSet);
  fscmFFSendDataFscmCFl(fscmFHeading);
  fscmFFSendDataFscmCFl(fscmFPitch);
  fscmFFSendDataFscmCFl(fscmFRoll);
  fscmFFSendDataFscmCBy(jly);
  fscmFFSendDataFscmCBy(jlx);
  fscmFFSendDataFscmCBy(jry);
  fscmFFSendDataFscmCBy(jrx);
  fscmFFSendDataFscmCBy(lk);
  fscmFFSendDataFscmCBy(rk);
  fscmFFSendDataFscmCBl(lt);
  fscmFFSendDataFscmCBl(rt);
  fscmFFSendDataFscmCFl(PXR);
  fscmFFSendDataFscmCFl(IXR);
  fscmFFSendDataFscmCFl(DXR);
  fscmFFSendDataFscmCFl(PYR);
  fscmFFSendDataFscmCFl(IYR);
  fscmFFSendDataFscmCFl(DYR);
  fscmFFSendDataFscmCFl(PZR);
  fscmFFSendDataFscmCFl(IZR);
  fscmFFSendDataFscmCFl(DZR);
  fscmFFSendDataFscmCFl(PXA);
  fscmFFSendDataFscmCFl(IXA);
  fscmFFSendDataFscmCFl(DXA);
  fscmFFSendDataFscmCFl(PYA);
  fscmFFSendDataFscmCFl(IYA);
  fscmFFSendDataFscmCFl(DYA);
}
void fscmFFDataToSendToFscmT() {
  fscmFFSendDataFscmTBl(fscmHomeSet);
  fscmFFSendDataFscmTBy(fscmFOriSystemCal);
  fscmFFSendDataFscmTBy(fscmFOriGyroCal);
  fscmFFSendDataFscmTBy(fscmFOriAccelCal);
  fscmFFSendDataFscmTBy(fscmFOriMagCal);
  fscmFFSendDataFscmTIn(fscmFDistMeters);
  fscmFFSendDataFscmTFl(fscmFHeadFmHome);
  fscmFFSendDataFscmTFl(fscmFOriQuatX);
  fscmFFSendDataFscmTFl(fscmFOriQuatY);
  fscmFFSendDataFscmTFl(fscmFOriQuatZ);
  fscmFFSendDataFscmTFl(fscmFOriQuatW);
  fscmFFSendDataFscmTFl(fscmFGpsLon);
  fscmFFSendDataFscmTFl(fscmFGpsLat);
  fscmFFSendDataFscmTFl(fscmFGpsSatStat);
  fscmFFSendDataFscmTFl(fscmFGpsSpeed);
  fscmFFSendDataFscmTFl(fscmFGpsHeading);
  fscmFFSendDataFscmTFl(fscmFGAlt);
  fscmFFSendDataFscmTFl(fscmFBatVolt);
  fscmFFSendDataFscmTIn(fscmFSigStrengthOfTran);
  fscmFFSendDataFscmTFl(fscmCPitch);
  fscmFFSendDataFscmTFl(fscmCRoll);
  fscmFFSendDataFscmTBy(fscmFWPI);
  fscmFFSendDataFscmTFl(fscmFWH);
  fscmFFSendDataFscmTFl(fscmFWD);
  fscmFFSendDataFscmTFl(fscmFWA);
}
void fscmFFDataToParseFromFscmT() {
  fscmRequestHomeSet = fscmFFParseDataFscmTBl();
  fscmFEnabled = fscmFFParseDataFscmTBy();
  jly = fscmFFParseDataFscmTBy();
  jlx = fscmFFParseDataFscmTBy();
  jry = fscmFFParseDataFscmTBy();
  jrx = fscmFFParseDataFscmTBy();
  lk = fscmFFParseDataFscmTBy();
  rk = fscmFFParseDataFscmTBy();
  lt = fscmFFParseDataFscmTBl();
  rt = fscmFFParseDataFscmTBl();
  pointsWNum = fscmFFParseDataFscmTBy();
  pointsWI = fscmFFParseDataFscmTBy();
  pointsWLon = fscmFFParseDataFscmTFl();
  pointsWLat = fscmFFParseDataFscmTFl();
  pointsWAlt = fscmFFParseDataFscmTFl();
  WAYPOINT_CLOSE_ENOUGH_DIST = fscmFFParseDataFscmTFl();
  PXR = fscmFFParseDataFscmTFl();
  IXR = fscmFFParseDataFscmTFl();
  DXR = fscmFFParseDataFscmTFl();
  PYR = fscmFFParseDataFscmTFl();
  IYR = fscmFFParseDataFscmTFl();
  DYR = fscmFFParseDataFscmTFl();
  PZR = fscmFFParseDataFscmTFl();
  IZR = fscmFFParseDataFscmTFl();
  DZR = fscmFFParseDataFscmTFl();
  PXA = fscmFFParseDataFscmTFl();
  IXA = fscmFFParseDataFscmTFl();
  DXA = fscmFFParseDataFscmTFl();
  PYA = fscmFFParseDataFscmTFl();
  IYA = fscmFFParseDataFscmTFl();
  DYA = fscmFFParseDataFscmTFl();

}
