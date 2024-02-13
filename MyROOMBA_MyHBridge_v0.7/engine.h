#ifndef ENGINE_H
#define ENGINE_H
#include "Arduino.h"

class Engine {
public:
 //  Lewy mot.   Prawy mot.
 // |FORW|BACK| |FORW|BACK| 
 // |BACK|FORW| |BACK|FORW|
Engine () : MAXSPEED(4095), MINSPEED(1024) { /* bazowy konstruktor*/ }
~Engine(){}

void motBegin (int LForw, int LBack, int RForw, int RBack) 
{
    LfPin=LForw;
    LbPin=LBack;
    RfPin=RForw;
    RbPin=RBack;
    pwm.setPWM(LfPin, 0, 0);
    pwm.setPWM(LbPin, 0, 0);
    pwm.setPWM(RfPin, 0, 0);
    pwm.setPWM(RbPin, 0, 0);
}

void setMaxSpeed(int spd)
{
  MAXSPEED = spd;
}
void setMinSpeed(int spd)
{
  MINSPEED = spd;
}

void drive(byte LV, byte RH )
{
  if (LV_p != LV || RH_p != RH)
  {
      // Przelicz kierunek
      if (RH < rAnaMin)
      {
        if (RH < AnaDWLim) sterL = GEARS[cGEAR];
        else sterL = map(RH, rAnaMin-1, AnaDWLim, 0, GEARS[cGEAR]);
        sterR = 0;
      }
      else if (RH > rAnaMax)
      {
        sterL = 0;
        if (RH > AnaUPLim) sterR = GEARS[cGEAR];
        else sterR = map(RH, rAnaMax+1, AnaUPLim, 0, GEARS[cGEAR]);
      }
      else { sterL = 0; sterR = 0; }


      // Do przodu/tylu
      if (LV > lAnaMax) // >50% -> FORWARD
      {
        pwm.setPWM(LbPin, 0, 0); // saveguard - nigdy Lf i Lb ON
        pwm.setPWM(RbPin, 0, 0);

        //spdL = map(LV, lAnaMax+1, 255, MINSPEED, MAXSPEED);;
        //spdR = map(LV, lAnaMax+1, 255, MINSPEED, MAXSPEED);;
        spdL = GEARS[cGEAR];
        spdR = GEARS[cGEAR];
        
        spdL = spdL - sterL;    // skret L/P
        if (spdL < 0) spdL = 0;
        spdR = spdR - sterR;
        if (spdR < 0) spdR = 0;
        
        pwm.setPWM(LfPin, 0, spdL );
        pwm.setPWM(RfPin, 0, spdR );

      }
      else if (LV < lAnaMin) // <50% -> REVERSE
      {
        pwm.setPWM(LfPin, 0, 0);
        pwm.setPWM(RfPin, 0, 0);

        //spdL = map(LV, lAnaMin-1, 0, MINSPEED, MAXSPEED);
        //spdR = map(LV, lAnaMin-1, 0, MINSPEED, MAXSPEED);
        spdL = GEARS[cGEAR];
        spdR = GEARS[cGEAR];
        
        spdL = spdL - sterL;    // skret L/P
        if (spdL < 0) spdL = 0;
        spdR = spdR - sterR;
        if (spdR < 0) spdR = 0;
        
        pwm.setPWM(LbPin, 0, spdL);
        pwm.setPWM(RbPin, 0, spdR);
      }
      
      // Stoi ale obraca
      else if (LV <= lAnaMax && LV >= lAnaMin)  // ~50% +- L/R -> BRAKE or <-L/R->
      {
        if (RH > rAnaMax) // obrot w prawo
        {
          pwm.setPWM(RfPin, 0, 0);          
          //spdR = map(RH, rAnaMax+1, 255, MINSPEED, MAXSPEED);
          spdR = GEARS[cGEAR];
          pwm.setPWM(RbPin, 0, spdR);

          pwm.setPWM(LbPin, 0, 0);          
          //spdL = map(RH, rAnaMax+1, 255, MINSPEED, MAXSPEED);
          spdL = GEARS[cGEAR];
          pwm.setPWM(LfPin, 0, spdL);
        }
        else if (RH < rAnaMin) // obrot w lewo
        {
          pwm.setPWM(RbPin, 0, 0);          
          //spdR = map(RH, rAnaMin-1, 0, MINSPEED, MAXSPEED);
          spdR = GEARS[cGEAR];
          pwm.setPWM(RfPin, 0, spdR);

          pwm.setPWM(LfPin, 0, 0);          
          //spdL = map(RH, rAnaMin-1, 0, MINSPEED, MAXSPEED);
          spdL = GEARS[cGEAR];
          pwm.setPWM(LbPin, 0, spdL);
        }
        else { this->stopMot(); }
      }
      // NIC, wylacz motory
      else{ this->stopMot(); }

    LV_p=LV; RH_p=RH;
  }
}

void stopMot()
{
  pwm.setPWM(LfPin, 0, 0);
  pwm.setPWM(LbPin, 0, 0);
  pwm.setPWM(RfPin, 0, 0);
  pwm.setPWM(RbPin, 0, 0);
}
void chngGear()
{
  if (cGEAR + 1 > 3) cGEAR = 0 ;
  else cGEAR++;
  //LCD_DATA[6] = cGEAR;
}
byte giveCGear()
{
  return this->cGEAR;
}
private:
int LfPin, LbPin, RfPin, RbPin;
byte LV_p, RH_p;
int sterL, sterR;
int spdL, spdR;
int MAXSPEED = 4095;
int MINSPEED = 1024;
const uint16_t GEARS[4] = { 1024, 2048, 3072, 4095 };
byte cGEAR = 3;
//int lMotMaxSpeed = 4095; // aby zrownac nieprawidlowosci motorow
//int rMotMaxSpeed = 4095;

// limity - histereza galek analogowych
const int lAnaMin = 120;
              // MID 132
const int lAnaMax = 144;
//--^Lvert------vRhor---
const int rAnaMin = 119;
              // MID 129
const int rAnaMax = 139;

const int AnaUPLim = 250;
const int AnaDWLim = 5;
};

#endif
