#ifndef RLEDS_H
#define RLEDS_H

class rleds
{
public:
  rleds(int pinR, int pinG, int pinB) : Rchan(pinR), Gchan(pinG), Bchan(pinB) 
  {
   _tics = 6; _plsON=100; _plsOFF=100;
   _iR=255; _iG=0; _iB=0;
  }
  ~rleds(){}
  void initLeds()
  { pwm.setPWM(Rchan, 0, 0); pwm.setPWM(Gchan, 0, 0); pwm.setPWM(Bchan, 0, 0); }
  
  void writeRGB(byte r, byte g, byte b)
  { _R=r; _G=g; _B=b; if (indiState==false) {pwm.setPWM(Rchan, 0, _R*16); pwm.setPWM(Gchan, 0, _G*16); pwm.setPWM(Bchan, 0, _B*16);} }

  void writeR(byte val) { _R = val; if (indiState==false) {pwm.setPWM(Rchan, 0, _R*16);} }
  void writeG(byte val) { _G = val; if (indiState==false) {pwm.setPWM(Gchan, 0, _G*16);} }
  void writeB(byte val) { _B = val; if (indiState==false) {pwm.setPWM(Bchan, 0, _B*16);} }

  void setIndiVals(int tics, int plsON, int plsOFF, byte r, byte g, byte b)
  {
      _tics=tics; _plsON=plsON; _plsOFF=plsOFF;
      _iR=r; _iG=g; _iB=b;
  }
  bool indicate()
  {
    if (indiState==false) { indiState = true; }

    if (ctick==false && indiState==true)
    {
      if (indiDel.del(_plsON))
      {
        pwm.setPWM(Rchan, 0, _iR*16); pwm.setPWM(Gchan, 0, _iG*16); pwm.setPWM(Bchan, 0, _iB*16);
        ctick=true;
        if (_tics > 0) { _tics--; }
      }
    }
    else if (ctick==true && indiState==true)
    {
      if (indiDel.del(_plsOFF))
      {
        pwm.setPWM(Rchan, 0, 0); pwm.setPWM(Gchan, 0, 0); pwm.setPWM(Bchan, 0, 0);
        ctick=false;
        if (_tics > 0) { _tics--; }
      }
    }

    // Przywruc oswietlenie
    
    if (_tics <= 0) 
    {
      indiState=false;
      pwm.setPWM(Rchan, 0, _R*16); pwm.setPWM(Gchan, 0, _G*16); pwm.setPWM(Bchan, 0, _B*16);
    }
    return indiState;
  }
  
  void writeHSV(byte h, byte s, byte v) // same 255
  {
    double  H, S, V, P, Q, T, RC, GC, BC, fract;
    H = 1.4117647058823529411764705882353*h; // mapuj 0-360 -> 0-255
    S = 0.0039215686274509803921568627451*s; //       0-1   -> 0-255
    V = 0.0039215686274509803921568627451*v; //       0-1   -> 0-255
    (H == 360.)?(H = 0.):(H /= 60.);
    fract = H - floor(H);
    P = V*(1. - S);
    Q = V*(1. - S*fract);
    T = V*(1. - S*(1. - fract));
    if (0. <= H && H < 1.) { RC = V; GC = T; BC = P; }
    else if (1. <= H && H < 2.) { RC = Q; GC = V; BC = P; }
    else if (2. <= H && H < 3.) { RC = P; GC = V; BC = T; }
    else if (3. <= H && H < 4.) { RC = P; GC = Q; BC = V; }
    else if (4. <= H && H < 5.) { RC = T; GC = P; BC = V; }
    else if (5. <= H && H < 6.) { RC = V; GC = P; BC = Q; }
    else { RC = 0; GC = 0; BC = 0; }
    byte xR = static_cast<byte>(round(RC*255));
    byte xG = static_cast<byte>(round(GC*255));
    byte xB = static_cast<byte>(round(BC*255));
    pwm.setPWM(Rchan, 0, xR*16); pwm.setPWM(Gchan, 0, xG*16); pwm.setPWM(Bchan, 0, xB*16);
  }
  
  private:
  int Rchan, Gchan, Bchan; // PCA PWM CHANS
  byte _R, _G, _B;
  
  Kronos indiDel;
  int _tics, _plsON, _plsOFF;
  byte _iR, _iG, _iB;
  bool indiState = false, ctick = false;
};
#endif
