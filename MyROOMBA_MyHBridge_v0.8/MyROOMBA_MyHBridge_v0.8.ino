
// Radio NRF
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(2, 3); // CE, CSN
const byte address[6] = "00008";

#include "Kronos.h"

#define SLAVE_ADDR 9 // ADRES I2C salve UNO+TFT
// Define Slave answer size
#define ANSWERSIZE 5

#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_FREQ 90

bool DEBUG = 0;
byte radioVals[8];
byte LVert{127},  LHor{127},  RVert{127},  RHor{127};
byte LVert_p{127},  LHor_p{127},  RVert_p{127},  RHor_p{127};

byte LbtnState = 0, RbtnState = 0, LbtnState_p = 0, RbtnState_p = 0;

byte potL, potR, potL_p, potR_p;

int SterL = 0, SterR = 0;
int SterFOR = 0, SterREV = 0;

//[i]     0     |   1   |    2   |    3   |     4     |    5    |   6    |            7                   |    8   |   9x    |
//    SIDEBRUSH | BRUSH | BLOWER | LIGHTS | CurrMOTOR | BATTERY | cGEAR  | BLCK(bit 0,1) invRev(bit 2)LSB | Lsonar | Rsonar |
byte LCD_DATA[10] = {0,0,0,0,0,0,3,0,0,0}; // 12-bit -> 4096 -> 8bit 256 -> coef: 16 

// PWM CH    0        1           2           3
uint16_t BLOWER{0}, BRUSH{0}, SIDEBRUSH{0}, LIGHTS{0};

// MAKS wartosci PWM / 4095
const uint16_t SBRUSH_MAXPWM = 2048; // 50% MAKS
const uint16_t BRUSH_MAXPWM  = 1020; // 25% MAKS
const uint16_t BLOWER_MAXPWM = 4020; // 98% MAKS
int Rblock = 5;
int Lblock = 6;
int LSonarPin{14}, RSonarPin{15};
int RevInvertPin = 7;
 bool revInvert = false, revInvert_p = false;
 bool RisBlck = false, RisBlck_p = false;
 bool LisBlck = false, LisBlck_p = false;
byte bitLRblck = 0;
byte selectedMotor = 0; // 0-all, 1-sidebrush, 2-brush, 3-blower
//enum MOTOR_TYPE { blower=0, brush=1, sidebrush=2 };
//MOTOR_TYPE cMotor = 0;

const uint16_t CAMSER_MIN = 210; // -- min(dolne), max(gorne) wychylenia serCam
const uint16_t CAMSER_MAX = 400;
uint16_t camSer=CAMSER_MIN;
Kronos camSerDel;

#include "engine.h"
Engine motors; // glowna instancja

void setup() {
Serial.begin(115200);

  // Radio
  if (radio.begin()) {Serial.println("RADIO ROBO OK --- Receiver!");}
  else Serial.println("RADIO ROBO ERROR !");
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_LOW);
  radio.powerUp();
  radio.enableDynamicPayloads();
  radio.setDataRate(RF24_250KBPS);
  radio.startListening(); 
  delay(100);
  //Wire.begin();

  Serial.println("Start PWM");
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);


  Serial.println("Start ENGINE");
  //            /__mot_L__\ /__mot_R__\
  //            AForw ARev | BForw BRev| PCAM*
  motors.motBegin(4,   5,     6,    7, &pwm);
  pinMode(17, INPUT); // A3 - V z bat(16,8v)
  pinMode(Rblock, INPUT_PULLUP);
  pinMode(Lblock, INPUT_PULLUP);
  pinMode(RevInvertPin, INPUT_PULLUP);
  pwm.setPWM(12, 0, camSer);
  Serial.println("SETUP END");
}

byte readUltrasonicDistance(int triggerPin, int echoPin)
{
  pinMode(triggerPin, OUTPUT);
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  pinMode(echoPin, INPUT);
  float CM = 0.01723*pulseIn(echoPin, HIGH, 10000); // czekaj tylko 10ms
  byte CMBYTE = static_cast<byte>(round(CM));
// Reads the echo pin, and returns the sound wave travel time in microseconds
  return CMBYTE;
}

void sendLCD()
{
  Wire.beginTransmission(SLAVE_ADDR);
  for (int i=0; i<sizeof(LCD_DATA); i++)
  { Wire.write(LCD_DATA[i]); }
  Wire.endTransmission();

  /* RESPONSE narazie nie potrzebne
  Serial.println("Receive data");
  // Read response from Slave
  // Read back 5 characters
  Wire.requestFrom(SLAVE_ADDR,ANSWERSIZE);
  
  // Add characters to string
  String response = "";
  while (Wire.available()) { char b = Wire.read(); response += b; }
  // Print to Serial Monitor
  Serial.println(response);
  */
}
//Kronos RVdel;
Kronos sndLcdDel;

void calcBat()
{
  int rawBat = analogRead(17);
  byte batVMapped = map(rawBat, 0, 1023, 0, 255);
  if (DEBUG) Serial.println("BatRAW: "+String(rawBat)+", mapped: "+String(batVMapped));
  LCD_DATA[5] = batVMapped;
}

void LRblockCheck()
{
    // Sprawdzanie blokady prawej i lewej
    digitalRead(Rblock)==LOW ? RisBlck=true : RisBlck=false;
    digitalRead(Lblock)==LOW ? LisBlck=true : LisBlck=false;
    digitalRead(RevInvertPin)==LOW ? revInvert=true : revInvert=false;
    // Akcja po blokadzie jednorazowe
    if (RisBlck_p != RisBlck)
    {
      if (RisBlck==true) { /*Serial.println("PRAWY BLOKADA");*/ bitLRblck|=0x1; } // bit LSB 0
      else { /*Serial.println("PRAWY WOLNY");*/ bitLRblck&=0xFE; }
      LCD_DATA[7] = bitLRblck;
      RisBlck_p = RisBlck;
    }
    if (LisBlck_p != LisBlck)
    {
      if (LisBlck==true) { /*Serial.println("PRAWY BLOKADA");*/ bitLRblck|=0x2; } // bit LSB 1
      else { /*Serial.println("PRAWY WOLNY");*/ bitLRblck&=0xFD;}
      LCD_DATA[7] = bitLRblck;
      LisBlck_p = LisBlck;
    }
    // Akcja: odwrucenie sterowania L/R 'do tylu'
    if (revInvert_p != revInvert)
    {
      if (revInvert==true) { bitLRblck|=0x4; } // bit LSB 2
      else { bitLRblck&=0xFB;}
      LCD_DATA[7] = bitLRblck;
      revInvert_p = revInvert;
    }
}

bool AItrig = false;
uint32_t AIstCounter = 0;
bool isAIinControll = false;
Kronos AIactivatingDel;
byte OBSTACLEDIS = 0;
Kronos AIatractor;

void AI_activateCheck()
{
  if (LHor < 15)
  { AItrig = true; }
  else { AItrig=false; AIstCounter=0; }
  
  if (LHor > 220)
  {
    AItrig = false;
    bitLRblck&=0xF7;
    isAIinControll = false;
    LCD_DATA[7] = bitLRblck;
  }

  if (AItrig==true && AIactivatingDel.del(100) )
  {
    AIstCounter++;
  }
  if (AIstCounter > 20)
  {
    bitLRblck|=0x8;
    isAIinControll = true;
    LCD_DATA[7] = bitLRblck;
    AIstCounter = 0;
    delay(500);
  }
}

void AI_drive()
{
  OBSTACLEDIS = readUltrasonicDistance(LSonarPin,LSonarPin);
  delay(50);
  if (OBSTACLEDIS < 3 || OBSTACLEDIS > 25 && (LisBlck==false && RisBlck==false))
  {
    if (AIatractor.del(random(2000, 5000))) {motors.drive(132, random(50,250)); delay(random(200,600)); }
    motors.drive(250, 129);
  }
  else if (OBSTACLEDIS >=3 && OBSTACLEDIS <= 25 || (LisBlck==true || RisBlck==true))
  {
    motors.drive(132, random(50,250));
    delay(random(300,900));
    motors.drive(132, 129);
    OBSTACLEDIS = readUltrasonicDistance(LSonarPin,LSonarPin);
    delay(300);
  }
  else if (LisBlck==true || RisBlck==true)
  {
    motors.drive(50, 129);
    delay(random(400,1000));
  }
  else motors.drive(132, 129); // STOP
}

void loop() {

  if (radio.available()) 
    {
      radio.read(&radioVals, sizeof(radioVals));
      // Analogi - ster.porusz.  BAZOWE co przychodza z pilota
      if (isAIinControll==false) LVert = radioVals[0];   // 132  ROBO: FORWARD/BACKWARD
      LHor  = radioVals[1];   // 127  - - - - (AI on/off)
      RVert = radioVals[2];   // 128  CAM: UP/DOWN
      if (isAIinControll==false) RHor  = radioVals[3];   // 129  ROBO: TURN LEFT/RIGHT
      // Buttony i poty
      LbtnState = radioVals[4]; // OFF/ON
      RbtnState = radioVals[5]; // OFF/ON
      potL = radioVals[6];      // plynne 0-255 POT L
      potR = radioVals[7];      // plynne 0-255 POT R

      if (isAIinControll==false) motors.drive(LVert, RHor);
      
      if (DEBUG==1)
      {
        Serial.print("LV: "+String(LVert)+", ");
        Serial.print("LH: "+String(LHor)+", ");
        Serial.print("RV: "+String(RVert)+", ");
        Serial.print("RH: "+String(RHor)+", ");
        Serial.print("Lbtn: "+String(LbtnState)+", ");
        Serial.println("Rbtn: "+String(RbtnState));
      }
      
    } // END RADIO AVAILABLE

    // A.I. Drive
    AI_activateCheck();
    if (isAIinControll==true) { AI_drive(); }


    // Sprawdzanie l/r blokady + inversRev
    LRblockCheck();
    
    //Zmiana L BTN --- wybor motora
    if (LbtnState_p != LbtnState)
    {
      selectedMotor++;
      if (selectedMotor > 3) selectedMotor = 0;
      LCD_DATA[4] = selectedMotor;
      LbtnState_p = LbtnState;
    }
    //Zmiana R BTN --- GEAR
    if (RbtnState_p != RbtnState)
    {
      motors.chngGear();
      LCD_DATA[6] = motors.giveCGear();
      RbtnState_p = RbtnState;
    }
    
    // Prawy POT - MOC motorow
    if (potR_p != potR)
    {
      /*Vout z PWM -> PWMmax(dla Vout) = Vout/(Vin/4095)*/
      switch(selectedMotor)
      {
        case 0: // WSZYSTKIE
        {
          if (potR < 15) 
          {
            SIDEBRUSH = 0; pwm.setPWM(0, 0, SIDEBRUSH);
            BRUSH = 0; pwm.setPWM(1, 0, BRUSH);
            BLOWER = 0; pwm.setPWM(2, 0, BLOWER);
          } 
          else  
          { 
            SIDEBRUSH = map(potR, 15, 255, 40, SBRUSH_MAXPWM); pwm.setPWM(0, 0, SIDEBRUSH);
            BRUSH =     map(potR, 15, 255, 40, BRUSH_MAXPWM);  pwm.setPWM(1, 0, BRUSH);
            BLOWER =    map(potR, 15, 255, 40, BLOWER_MAXPWM); pwm.setPWM(2, 0, BLOWER);
          }
          
          LCD_DATA[0] = map(SIDEBRUSH, 0, SBRUSH_MAXPWM, 0, 255);
          LCD_DATA[1] = map(BRUSH, 0, BRUSH_MAXPWM, 0, 255);
          LCD_DATA[2] = map(BLOWER, 0, BLOWER_MAXPWM, 0, 255);
          
          potR_p = potR;
          break;
        }
        case 1: // SIDEBRUSH
        {
          if (potR < 15) { SIDEBRUSH = 0; pwm.setPWM(0, 0, SIDEBRUSH); } // ~11v
          else  { SIDEBRUSH = map(potR, 15, 255, 40, SBRUSH_MAXPWM); pwm.setPWM(0, 0, SIDEBRUSH); }
          LCD_DATA[0] = map(SIDEBRUSH, 0, SBRUSH_MAXPWM, 0, 255);
          potR_p = potR;
          break;
        }
        case 2: // MAIN BRUSH - Vout = 12.0v ~ PWMmax = 3071 / dla 6.0v = 1535
        {
          if (potR < 15) { BRUSH = 0; pwm.setPWM(1, 0, BRUSH); } // ~16v
          else  { BRUSH = map(potR, 15, 255, 40, BRUSH_MAXPWM); pwm.setPWM(1, 0, BRUSH); }
          LCD_DATA[1] = map(BRUSH, 0, BRUSH_MAXPWM, 0, 255);
          potR_p = potR;
          break;
        }
        case 3: // BLOWER - Vout = 15.0v ~ PWMmax = 3839
        {
          if (potR < 15) { BLOWER = 0; pwm.setPWM(2, 0, BLOWER); } // ~ 16v
          else  { BLOWER = map(potR, 15, 255, 40, BLOWER_MAXPWM); pwm.setPWM(2, 0, BLOWER); }
          LCD_DATA[2] = map(BLOWER, 0, BLOWER_MAXPWM, 0, 255);
          potR_p = potR;
          break;
        }
        default: break;
      } // switch
    } // if
    
    // Lewy POT - headlights
    if (potL_p != potL)
    {
      if (potL < 15) { LIGHTS = 0; pwm.setPWM(3, 0, LIGHTS); }
      else           { LIGHTS = map(potL, 15, 255, 512, 4095); pwm.setPWM(3, 0, LIGHTS); }
      LCD_DATA[3] = map(LIGHTS, 0, 4095, 0, 255);
      potL_p = potL;
    }

    //Wysylanie na EKRAN
    if (sndLcdDel.del(200))
    { 
      calcBat();
      LCD_DATA[8] = readUltrasonicDistance(LSonarPin,LSonarPin);
      //LCD_DATA[9] = readUltrasonicDistance(RSonarPin,RSonarPin);
      //Serial.println("Sonar L: "+String(LCD_DATA[8]));
      //Serial.println("Sonar R: "+String(LCD_DATA[9]));
      sendLCD();
    }
    
    
    /* //Zmiana LEWA galka HORYZONTALNA - narazie NIC
    if (LHor > 220 && RVdel.del(40)){}
    if (LHor < 35 && RVdel.del(40)){}
    */

    // Zmiana PRAWA galka VERTYKALNA - U/D camera
    // PWM down 210, up 400
    if (RVert > 220 && camSerDel.del(30))
    {
      pwm.setPWM(12, 0, camSer);
      if (camSer<400) camSer+=4;
      Serial.println("Cam ser: "+String(camSer));
    }
    if (RVert < 30 && camSerDel.del(30))
    {
      pwm.setPWM(12, 0, camSer);
      if (camSer>210) camSer-=4;
      Serial.println("Cam ser: "+String(camSer));
    }
    
    
} // loop END
