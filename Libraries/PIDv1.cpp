//  includes
#include <PIDv1.h>

//  variable declarations
int sampleTime = 1000;
int nVars = 0;
bool inAuto = false;

//  declare arrays
double Input[NUMBER_OF_VARIABLES];
double Output[NUMBER_OF_VARIABLES];
double lastPV[NUMBER_OF_VARIABLES];
double I[NUMBER_OF_VARIABLES];
unsigned long tik[NUMBER_OF_VARIABLES];

const int f = NUMBER_OF_VARIABLES*3;
double tunePs[f];
const int g = NUMBER_OF_VARIABLES*2;
double outMin[g];
double outMax[g];

int controllerDirection = DIRECT;

void  setControllerDirection (int Direction) {
  controllerDirection = Direction;
}

void  setOutputLimits(double Min, double Max) {
  if(Min > Max) return;
  outMin[(nVars*2)] = Min;
  outMax[(nVars*2) + 1] = Max;
  nVars++;
}

void  setMode(int Mode) {
  bool newAuto = (Mode == AUTOMATIC);
  if(newAuto && !inAuto) {
     Initialize();
  }
  inAuto = newAuto;
}

int c1 = 0;
void  setNewSampleTime(int NewSampleTime) {
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)sampleTime;
      tunePs[nVars*c1 + 1] *= ratio;
      tunePs[nVars*c1 + 2] /= ratio;
      sampleTime = (unsigned long)NewSampleTime;
   }
}

int c2 = 0;
void  setTunings(double Kp, double Ki, double Kd){

  if(Kp < 0 || Ki < 0 || Kd < 0) return;
  
  double SampleTimeInSec = double(sampleTime/1000.0);
  
  tunePs[nVars*c2] = Kp;
  tunePs[nVars*c2 + 1] = Ki * SampleTimeInSec;
  tunePs[nVars*c2 + 2] = Kd / SampleTimeInSec;

  if(controllerDirection == REVERSE){
    tunePs[nVars*c2] = (0 - tunePs[nVars*c2]);
    tunePs[nVars*c2 + 1] = (0 - tunePs[nVars*c2 + 2]);
    tunePs[nVars*c2 + 2] = (0 - tunePs[nVars*c2 + 2]);
  }
  c2++;
}

//  compute has to be called in the same order as the tuning parameters were set in the main code
//  ie tuning parameters were set as: 0 - Pitch, 1 - Roll, 2 - Yaw, then call compute in that same order
//
int c3 = 0;
int  compute(int sP, int pV) {

  if(!inAuto) return 999;
  unsigned long tok = millis();
  int tiktok = tok - tik[c3];
  
  if(tiktok >= sampleTime){
    double kp, ki, kd, P, D;
    
    kp = tunePs[nVars*c3];
    ki = tunePs[nVars*c3 + 1];
    kd = tunePs[nVars*c3 + 2];
    
    double error = sP - pV;
    I[c3] += ki*error;
    double dPV = (pV - lastPV[c3]);
    
    P = kp*error;
    D = -kd*dPV;

    Output[c3] = P + I[c3] + D;
    if(Output[c3] > outMax[c3*2 + 1]){
      I[c3] -= Output[c3] - outMax[c3*2 + 1];
      Output[c3] = outMax[c3*2 + 1];
    }
    else if(Output[c3] < outMin[c3*2]){
      I[c3] += outMin[c3*2] - Output[c3];
      Output[c3] = outMin[c3*2];
    }
    
    Input[c3] += Output[c3];
    lastPV[c3] = Input[c3];
    tik[c3] = tok;
    
    Input[c3] += 0.5;
    int result = (int) Input[c3];
//Serial.println(result);
    return result;
  }
  c3++;
  if(c3 > nVars) c3 = 0;
}

void  Initialize() {
  for(int i = 0; i < nVars; i++) {
    lastPV[i] = Input[i];
    I[i] = Output[i];
    if(I[i] > outMax[i*2 + 1]) I[i] = outMax[i*2 + 1];
    else if(I[i] < outMin[i*2]) I[i] = outMin[i*2];  
  }
}
