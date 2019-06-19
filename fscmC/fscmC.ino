#include "fscmCFunctions.h"
#include "fscmC.h"
void setup() {
  fscmCFSetupGyro();
  Serial2.begin(250000);//to fscmF
  Serial.begin(2000000);//for DEBUG
  pinMode(13, OUTPUT);
  pinMode(8, INPUT_PULLUP);
  zeroGyro();
  leftFront.attach(leftFrontPin);
  rightFront.attach(rightFrontPin);
  leftBack.attach(leftBackPin);
  rightBack.attach(rightBackPin);
  leftFront.writeMicroseconds(1000);
  rightFront.writeMicroseconds(1000);
  leftBack.writeMicroseconds(1000);
  rightBack.writeMicroseconds(1000);
}
void loop() {
  fscmCFReadGyro();
  fscmCFFscmFComms();
  if (fscmHomeSet) {
    zeroGyro();
  }
  if (millis() - lastRecievedFscmF < SHUTOFF_AFTER_MILLIS) {
    if (fscmCEnabled) {
      digitalWrite(13, HIGH);
      ////////////enabled
      if (modeVal == 1) {//manual
        throttleOutput = map(jly, 0, 255, 0, 1000);
        yawOutput = map(jlx, 0, 255, -250, 250);
        rollOutput = map(jrx, 0, 255, -250, 250);
        pitchOutput = map(jry, 0, 255, -300, 300);
      }
      if (modeVal == 2) {//rotation
        PIDXR.SetTunings(PXR, IXR, DXR);
        PIDYR.SetTunings(PYR, IYR, DYR);
        PIDZR.SetTunings(PZR, IZR, DZR);
        PIDXR.SetMode(AUTOMATIC);
        PIDYR.SetMode(AUTOMATIC);
        PIDZR.SetMode(AUTOMATIC);
        PIDXR.SetSampleTime(10);
        PIDYR.SetSampleTime(10);
        PIDZR.SetSampleTime(10);
        PIDXR.SetOutputLimits(-120, 120);
        PIDYR.SetOutputLimits(-120, 100);
        PIDZR.SetOutputLimits(-120, 120);
        XRS = mapf(jry, 0, 255, -60.000, 60.000);
        YRS = mapf(jrx, 0, 255, -60.000, 60.000);
        ZRS = mapf(jlx, 0, 255, -60.000, 60.000);
        PIDXR.Compute();
        PIDYR.Compute();
        PIDZR.Compute();
        throttleOutput = map(jly, 0, 255, 0, 1000);
        yawOutput = ZRO;
        rollOutput = YRO;
        pitchOutput = XRO;
      }
      if (modeVal == 3) {//angle
        PIDXA.SetTunings(PXR, IXR, DXR);
        PIDYA.SetTunings(PYR, IYR, DYR);
        PIDZR.SetTunings(PZR, IZR, DZR);
        PIDXA.SetMode(AUTOMATIC);
        PIDYA.SetMode(AUTOMATIC);
        PIDZR.SetMode(AUTOMATIC);
        PIDXA.SetSampleTime(10);
        PIDYA.SetSampleTime(10);
        PIDZR.SetSampleTime(10);
        PIDXA.SetOutputLimits(-120, 120);
        PIDYA.SetOutputLimits(-120, 120);
        PIDZR.SetOutputLimits(-120, 120);
        XAS = mapf(jry, 0, 255, -35.000, 35.000);
        YAS = mapf(jrx, 0, 255, -35.000, 35.000);
        ZRS = mapf(jlx, 0, 255, -60.000, 60.000);
        PIDXA.Compute();
        PIDYA.Compute();
        PIDZR.Compute();
        throttleOutput = map(jly, 0, 255, 0, 1000);
        yawOutput = ZRO;
        rollOutput = YAO;
        pitchOutput = XAO;
      }
      rightFrontOutput = throttleOutput - rollOutput + pitchOutput + yawOutput;
      leftFrontOutput = throttleOutput + rollOutput + pitchOutput - yawOutput;
      rightBackOutput = throttleOutput - rollOutput - pitchOutput - yawOutput;
      leftBackOutput = throttleOutput + rollOutput - pitchOutput + yawOutput;
      leftFront.writeMicroseconds(constrain(leftFrontOutput + 1000, 1000, 2000));
      rightBack.writeMicroseconds(constrain(rightBackOutput + 1000, 1000, 2000));
      rightFront.writeMicroseconds(constrain(rightFrontOutput + 1000, 1000, 2000));
      leftBack.writeMicroseconds(constrain(leftBackOutput + 1000, 1000, 2000));
      digitalWrite(13, LOW);
    } else {
      digitalWrite(13, LOW);
      ///////////disabled
      leftFront.writeMicroseconds(1000);
      rightBack.writeMicroseconds(1000);
      rightFront.writeMicroseconds(1000);
      leftBack.writeMicroseconds(1000);
    }
  }
  else {//lost connection
    digitalWrite(13, (millis() / 250) % 2);
    //lost connection (disable)
  }
}
void fscmCFDataToSendToFscmF() {
  fscmCFSendDataFscmFFl(fscmCPitch);
  fscmCFSendDataFscmFFl(fscmCRoll);
}
void fscmCFDataToParseFromFscmF() {
  fscmCEnabled = fscmCFParseDataFscmFBl();
  fscmHomeSet = fscmCFParseDataFscmFBl();
  fscmFHeading = fscmCFParseDataFscmFFl();
  fscmFPitch = fscmCFParseDataFscmFFl();
  fscmFRoll = fscmCFParseDataFscmFFl();
  jly = fscmCFParseDataFscmFBy();
  jlx = fscmCFParseDataFscmFBy();
  jry = fscmCFParseDataFscmFBy();
  jrx = fscmCFParseDataFscmFBy();
  lk = fscmCFParseDataFscmFBy();
  rk = fscmCFParseDataFscmFBy();
  lt = fscmCFParseDataFscmFBl();
  rt = fscmCFParseDataFscmFBl();
  PXR = fscmCFParseDataFscmFFl();
  IXR = fscmCFParseDataFscmFFl();
  DXR = fscmCFParseDataFscmFFl();
  PYR = fscmCFParseDataFscmFFl();
  IYR = fscmCFParseDataFscmFFl();
  DYR = fscmCFParseDataFscmFFl();
  PZR = fscmCFParseDataFscmFFl();
  IZR = fscmCFParseDataFscmFFl();
  DZR = fscmCFParseDataFscmFFl();
  PXA = fscmCFParseDataFscmFFl();
  IXA = fscmCFParseDataFscmFFl();
  DXA = fscmCFParseDataFscmFFl();
  PYA = fscmCFParseDataFscmFFl();
  IYA = fscmCFParseDataFscmFFl();
  DYA = fscmCFParseDataFscmFFl();
}
double mapf(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
