#define SHUTOFF_AFTER_MILLIS 1500
#include <PID_v1.h>
#include <Servo.h>

#define leftFrontPin 16
#define leftBackPin 17
#define rightFrontPin 22
#define rightBackPin 23
Servo leftFront;
Servo rightFront;
Servo leftBack;
Servo rightBack;
byte modeVal = 1;
byte jly = 0;
byte jlx = 0;
byte jry = 0;
byte jrx = 0;
byte lk = 0;
byte rk = 0;
boolean lt = false;
boolean rt = false;
float fscmFGAlt = 0.000;
float fscmFHeading, fscmFPitch, fscmFRoll = 0.000;

double PXR = 0.0000;
double IXR = 0.0000;
double DXR = 0.0000;
double PYR = 0.0000;
double IYR = 0.0000;
double DYR = 0.0000;
double PZR = 0.0000;
double IZR = 0.0000;
double DZR = 0.0000;
double PXA = 0.0000;
double IXA = 0.0000;
double DXA = 0.0000;
double PYA = 0.0000;
double IYA = 0.0000;
double DYA = 0.0000;

double XRS = 0.0000;
double YRS = 0.0000;
double ZRS = 0.0000;
double XRO = 0.0000;
double YRO = 0.0000;
double ZRO = 0.0000;

double XAS = 0.0000;
double YAS = 0.0000;
double XAO = 0.0000;
double YAO = 0.0000;

double rollOutput = 0.00;
double pitchOutput = 0.00;
double throttleOutput = 0.00;
double yawOutput = 0.00;

int rightFrontOutput = 0;
int rightBackOutput = 0;
int leftFrontOutput = 0;
int leftBackOutput = 0;

PID PIDXR(&GDSX, &XRO, &XRS, PXR, IXR, DXR, DIRECT);
PID PIDYR(&GDSY, &YRO, &YRS, PYR, IYR, DYR, DIRECT);
PID PIDZR(&GDSZ, &ZRO, &ZRS, PZR, IZR, DZR, DIRECT);
PID PIDXA(&fscmCPitch, &XAO, &XAS, PXA, IXA, DXA, DIRECT);
PID PIDYA(&fscmCRoll, &YAO, &YAS, PYA, IYA, DYA, DIRECT);
