#include <Servo.h>
Servo wheel;
Servo arm;

// Radio NRF
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(2, 3);
const byte address[6] = "00008";
#include "Kronos.h"

bool DEBUG = 0;

#define ATop 7
#define ABot 6
#define BTop 8
#define BBot 5

int sCLK = 17;
int sLCH = 18;
int sDAT = 19;
bool isIndLed = false;
bool indiDir = 1;
Kronos indiDel;
int indiIter = 0;
  // nizej nie rusza    zmienna
byte MinSPD = 75,     MaxSPD = 250, MaxSPDp = 250;

void shiftLed(byte mi, byte ma)
{
  byte gear = map(ma, mi, 250, 0, 8);
  digitalWrite(sLCH, LOW);
  for (int i=0; i<8; i++)
  {
    if (gear > i ) digitalWrite(sDAT, HIGH);
    else digitalWrite(sDAT, LOW);
    digitalWrite(sCLK,HIGH);
    digitalWrite(sCLK,LOW);
    
  }
  digitalWrite(sLCH, HIGH);
}

void setup() {
  pinMode(ATop, OUTPUT);
  pinMode(ABot, OUTPUT);
  pinMode(BTop, OUTPUT);
  pinMode(BBot, OUTPUT);
  digitalWrite(ATop, LOW);
  digitalWrite(BTop, LOW);
  analogWrite(ABot, 0);
  analogWrite(BBot, 0);

  pinMode(sCLK, OUTPUT);
  pinMode(sLCH, OUTPUT);
  pinMode(sDAT, OUTPUT);
  shiftLed(MinSPD, MinSPD);
  
  wheel.attach(10);
  wheel.write(90);    // 70 - 110 Deg.
  arm.attach(9);
  arm.write(20);      // 40 - 0 Deg.
  
  Serial.begin(115200);

  // Radio
  if (radio.begin()) {Serial.println("RADIO OK --- Receiver!");}
  else Serial.println("RADIO ERROR !");
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_LOW);
  radio.powerUp();
  radio.enableDynamicPayloads();
  radio.setDataRate(RF24_250KBPS);
  radio.startListening(); 
  delay(100);
}

byte LVert{127},  LHor{127},  RVert{127},  RHor{127};
byte LbtnState = 0, RbtnState = 0;
byte LbtnState_p = 0, RbtnState_p = 0;
byte potL{0}, potR{0};
byte radioVals[8];

void loop() {
byte steSer = 90;
byte armSer = 20;
if (radio.available()) 
  {
    radio.read(&radioVals, sizeof(radioVals));
    LVert = radioVals[0];
    LHor = radioVals[1];
    RVert = radioVals[2];
    RHor = radioVals[3];
    LbtnState = radioVals[4];
    RbtnState = radioVals[5];
    potL = radioVals[6];
    potR = radioVals[7];

    if (LbtnState_p==0)
    {
      armSer = map(RVert, 0, 255, 40, 0);
      arm.write(armSer);
    }

    steSer = map(RHor, 0, 255, 70, 110);
    wheel.write(steSer);

    // Biegi z pota
    MaxSPD = map(potR, 0, 255, MinSPD, 255);
    if (MaxSPDp != MaxSPD)
    {
      if (isIndLed == false) shiftLed(MinSPD, MaxSPD);
      MaxSPDp = MaxSPD;
    }

    if (LbtnState_p != LbtnState)
    {
      LbtnState_p = LbtnState;
      isIndLed = true;
    }
    // R btn narazie nie uzywany
    /*
    if (RbtnState_p != RbtnState) {}
    */
    
  /* moj hmostek - ok
   * |7|8|
   * |5|6|  ->  
   *        FOR: 7on 6pwm -> 8off 5off
   *        REV: 8on 5pwm -> 7off 6off
   *    #ATop 7 #BTop 8
        #BBot 5 #ABot 6   
   */

    if (LVert > 135)
    {
      digitalWrite(BTop, LOW); digitalWrite(BBot, LOW); digitalWrite(ATop, HIGH);
      analogWrite (ABot, map(LVert, 136, 255, MinSPD, MaxSPD) );
    }
    else if (LVert < 125)
    {
      digitalWrite(ATop, LOW); digitalWrite(ABot, LOW); digitalWrite(BTop, HIGH);
      analogWrite (BBot, map(LVert, 124, 0, MinSPD, MaxSPD) );
    }
    else
    { digitalWrite(ATop, LOW); digitalWrite(ABot, LOW); digitalWrite(BTop, LOW); digitalWrite(BBot, LOW); }
    if (DEBUG)
    {
      Serial.print("LV: "+String(LVert)+", ");
      Serial.print("LH: "+String(LHor)+", ");
      Serial.print("RV: "+String(RVert)+", ");
      Serial.print("RH: "+String(RHor)+", ");
      Serial.print("Lbtn: "+String(LbtnState)+", ");
      Serial.println("Rbtn: "+String(RbtnState)); 
    }
  } // RADIO AVAIL

  if (isIndLed==true && indiDel.del(200))
  {
    if (LbtnState_p==0){
      if (indiDir) { shiftLed(0, 64); indiIter++; indiDir=!indiDir;}
      else { shiftLed(0, 0); indiIter++; indiDir=!indiDir;}}

    if (LbtnState_p==1){
      if (indiDir) { shiftLed(0, 250); indiIter++; indiDir=!indiDir;}
      else { shiftLed(0, 0); indiIter++; indiDir=!indiDir;}}
      
    if (indiIter>8)
    {
      indiIter = 0;
      isIndLed = false;
    }
  }

}
