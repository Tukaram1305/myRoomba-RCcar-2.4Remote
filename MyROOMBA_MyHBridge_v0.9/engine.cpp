#include "engine.h"

Engine::Engine() : MAXSPEED(4095), MINSPEED(1024) {}
Engine::Engine(uint16_t MIspd, uint16_t MAspd) : MAXSPEED(MAspd), MINSPEED(MIspd) {}
Engine::~Engine(){ this->pwmptr=nullptr; }
void Engine::motBegin(int LForw, int LBack, int RForw, int RBack, Adafruit_PWMServoDriver *adaPwmPtr)
{
    pwmptr = adaPwmPtr;
    LfPin=LForw;
    LbPin=LBack;
    RfPin=RForw;
    RbPin=RBack;
    pwmptr->setPWM(LfPin, 0, 0);
    pwmptr->setPWM(LbPin, 0, 0);
    pwmptr->setPWM(RfPin, 0, 0);
    pwmptr->setPWM(RbPin, 0, 0);
}
void Engine::setMaxSpeed(int spd)
{
  MAXSPEED = spd;
}
void Engine::setMinSpeed(int spd)
{
  MINSPEED = spd;
}
void Engine::drive(byte LV, byte RH )
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
        pwmptr->setPWM(LbPin, 0, 0); // saveguard - nigdy Lf i Lb ON
        pwmptr->setPWM(RbPin, 0, 0);

        //spdL = map(LV, lAnaMax+1, 255, MINSPEED, MAXSPEED);
        //spdR = map(LV, lAnaMax+1, 255, MINSPEED, MAXSPEED);
        spdL = GEARS[cGEAR];
        spdR = GEARS[cGEAR];
        
        spdL = spdL - sterL;    // skret L/P
        if (spdL < 0) spdL = 0;
        spdR = spdR - sterR;
        if (spdR < 0) spdR = 0;

        if (LisBlck==false && RisBlck==false)
        {
          pwmptr->setPWM(LfPin, 0, spdL );
          pwmptr->setPWM(RfPin, 0, spdR );
        }
        else
        {
          pwmptr->setPWM(LfPin, 0, 0 );
          pwmptr->setPWM(RfPin, 0, 0 );
        }

      }
      else if (LV < lAnaMin) // <50% -> REVERSE
      {
        pwmptr->setPWM(LfPin, 0, 0);
        pwmptr->setPWM(RfPin, 0, 0);

        //spdL = map(LV, lAnaMin-1, 0, MINSPEED, MAXSPEED);
        //spdR = map(LV, lAnaMin-1, 0, MINSPEED, MAXSPEED);
        spdL = GEARS[cGEAR];
        spdR = GEARS[cGEAR];

        if (revInvert==false)
        {
          spdL = spdL - sterL;    // skret L/P - normalnie
          if (spdL < 0) spdL = 0;
          spdR = spdR - sterR;
          if (spdR < 0) spdR = 0;
        }
        else
        {
          spdL = spdL - sterR;    // skret L/P - normalnie
          if (spdL < 0) spdL = 0;
          spdR = spdR - sterL;
          if (spdR < 0) spdR = 0;
        }
        
        pwmptr->setPWM(LbPin, 0, spdL);
        pwmptr->setPWM(RbPin, 0, spdR);
      }
      
      // Stoi ale obraca
      else if (LV <= lAnaMax && LV >= lAnaMin)  // ~50% +- L/R -> BRAKE or <-L/R->
      {
        if (RH > rAnaMax) // obrot w prawo
        {
          pwmptr->setPWM(RfPin, 0, 0);          
          //spdR = map(RH, rAnaMax+1, 255, MINSPEED, MAXSPEED);
          spdR = GEARS[cGEAR];
          pwmptr->setPWM(RbPin, 0, spdR);

          pwmptr->setPWM(LbPin, 0, 0);          
          //spdL = map(RH, rAnaMax+1, 255, MINSPEED, MAXSPEED);
          spdL = GEARS[cGEAR];
          pwmptr->setPWM(LfPin, 0, spdL);
        }
        else if (RH < rAnaMin) // obrot w lewo
        {
          pwmptr->setPWM(RbPin, 0, 0);          
          //spdR = map(RH, rAnaMin-1, 0, MINSPEED, MAXSPEED);
          spdR = GEARS[cGEAR];
          pwmptr->setPWM(RfPin, 0, spdR);

          pwmptr->setPWM(LfPin, 0, 0);          
          //spdL = map(RH, rAnaMin-1, 0, MINSPEED, MAXSPEED);
          spdL = GEARS[cGEAR];
          pwmptr->setPWM(LbPin, 0, spdL);
        }
        else { this->stopMot(); }
      }
      // NIC, wylacz motory
      else{ this->stopMot(); }

    LV_p=LV; RH_p=RH;
  }
}
void Engine::stopMot()
{
  pwmptr->setPWM(LfPin, 0, 0);
  pwmptr->setPWM(LbPin, 0, 0);
  pwmptr->setPWM(RfPin, 0, 0);
  pwmptr->setPWM(RbPin, 0, 0);
}
void Engine::chngGear()
{if (cGEAR + 1 > 3) cGEAR = 0 ;
 else cGEAR++; }
byte Engine::giveCGear()
{ return this->cGEAR; }
