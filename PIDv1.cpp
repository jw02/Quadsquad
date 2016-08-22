//  includes
#include <PIDv1.h>

//  variable declarations
unsigned long tik;
double Input, Output, setPoint;
double lastPV;
double kp, ki, kd, P, I, D;
int sampleTime = 1000;
double outMin, outMax;
bool inAuto = false;

int controllerDirection = DIRECT;

void setControllerDirection (int Direction){
  controllerDirection = Direction;
}

void Initialize() {
  lastPV = Input;
  I = Output;
  if(I > outMax) I = outMax;
  else if(I < outMin) I = outMin;  
}

void setMode(int Mode){
  bool newAuto = (Mode == AUTOMATIC);
  if(newAuto && !inAuto) {
    Initialize();
  }
  inAuto = newAuto;
}

void setOutputLimits(double Min, double Max) {
  if(Min > Max) return;
  outMin = Min;
  outMax = Max;

  if(Output > outMax) Output = outMax;
  else if(Output < outMin) Output = outMin;

  if(I > outMax) I = outMax;
  else if(I < outMin) I = outMin;
}

void setNewSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)sampleTime;
      ki *= ratio;
      kd /= ratio;
      sampleTime = (unsigned long)NewSampleTime;
   }
}

void setTunings(double Kp, double Ki, double Kd){

  if(Kp < 0 || Ki < 0 || Kd < 0) return;
  
  double SampleTimeInSec = double(sampleTime/1000.0);
  
  kp = Kp;
  ki = Ki * SampleTimeInSec;
  kd = Kd / SampleTimeInSec;

  if(controllerDirection == REVERSE){
    kp = (0 - kp);
    ki = (0 - ki);
    kd = (0 - kd);
  }
}

void compute(double sP, double pV){
  
  if(!inAuto) return;
  unsigned long tok = millis();
  int tiktok = tok - tik;
  
  if(tiktok >= sampleTime){
    //Serial.println(pV);
    double error = sP - pV;
    I += ki*error;
    double dPV = (pV - lastPV);
        
    P = kp*error;
    D = -kd*dPV;

    /**
    Serial.print(error);
    Serial.print(" ");
    Serial.print(errSum);
    Serial.print(" ");
    Serial.println(dErr);
    /**/
    
    Output = P + I + D;
    if(Output > outMax){
      I -= Output - outMax;
      Output = outMax;
    }
    else if(Output < outMin){
      I += outMin - Output;
      Output = outMin;
    }
    
    //Serial.println(Output);
    Input += Output;
    Serial.println(Input);
    lastPV = Input;
    tik = tok;
  }
}
