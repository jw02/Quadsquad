//  includes
#include <PIDv1.h>

//  variable declarations
int sampleTime = 10;
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
const int g = NUMBER_OF_VARIABLES;
double outMin[g];
double outMax[g];

int controllerDirection = DIRECT;

void  setControllerDirection (int Direction) {
  controllerDirection = Direction;
}

void  setOutputLimits(double Min, double Max) {
  if(Min > Max) return;
  outMin[nVars] = Min;
  outMax[nVars] = Max;
  nVars++;
}

void  setMode(int Mode) {
//  arbitrarily chose setMode( ) to set all arrays equal to zeros
memset(Input, 0, sizeof(Input));
memset(Output, 0, sizeof(Output));
memset(lastPV, 0, sizeof(lastPV));
memset(I, 0, sizeof(I));
memset(tik, 0, sizeof(tik));
memset(tunePs, 0, sizeof(tunePs));
memset(outMin, 0, sizeof(outMin));
memset(outMax, 0, sizeof(outMax));

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
  /**
 Serial.print("Controller: ");
 Serial.println(sP);
 Serial.println("/n");
 Serial.println("MPU: ");
 Serial.print(pV);
 Serial.println("/n");
  /**/
 
  if(!inAuto) return 999;
  unsigned long tok = millis();
  int tiktok = tok - tik[c3];
  
  if(tiktok >= sampleTime){
    double kp, ki, kd, P, D;
    
    kp = tunePs[nVars*c3];
    ki = tunePs[nVars*c3 + 1];
    kd = tunePs[nVars*c3 + 2];

    /**
    Serial.print("kP = ");
    Serial.println(kp);
    Serial.print("kI = ");
    Serial.println(ki);
    Serial.print("kD = ");
    Serial.println(kd);
    /**/
    
    double error = sP - pV;
    /**
    Serial.print("Error: ");
    Serial.print(sP);
    Serial.print(" - ");
    Serial.println(pV);
    /**/
    
    I[c3] += ki*error;
    double dPV = (pV - lastPV[c3]);
    //Serial.print("lastPV = ");
    //Serial.println(lastPV[c3]);
    
    P = kp*error;
    D = -kd*dPV;
    
    /**
    Serial.print("P = ");
    Serial.println(P);
    Serial.print("I = ");
    Serial.println(I[c3]);
    Serial.print("D = ");
    Serial.println(D);
    /**/
    
    Output[c3] = P + I[c3] + D;
    
    /**
    Serial.print("Output[");
    Serial.print(c3);
    Serial.print("]: ");
    Serial.println(Output[c3]);
    /**/

    /**
    if(Output[c3] > outMax[c3]){
      I[c3] -= Output[c3] - outMax[c3];
      Output[c3] = outMax[c3];
    }
    else if(Output[c3] < outMin[c3]){
      I[c3] += outMin[c3] - Output[c3];
      Output[c3] = outMin[c3];
    }
    /**/
    
    //Input[c3] += Output[c3];
    lastPV[c3] = Input[c3];
    tik[c3] = tok;
    
    //Input[c3] += 0.5;
    int result = (int) Output[c3];
    
    c3++;
    if(c3 > nVars) c3 = 0;
    
   // Serial.println(result);
    return result;
  }
  else {
    return 0.0;
  }
}

void  Initialize() {
  for(int i = 0; i < nVars; i++) {
    lastPV[i] = Input[i];
    I[i] = Output[i];
    if(I[i] > outMax[i]) I[i] = outMax[i];
    else if(I[i] < outMin[i]) I[i] = outMin[i];  
  }
}
