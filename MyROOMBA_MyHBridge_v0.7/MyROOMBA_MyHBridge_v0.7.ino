
// Radio NRF
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(2, 3); // CE, CSN
const byte address[6] = "00008";

#include "Kronos.h"

// Define Slave I2C Address
#define SLAVE_ADDR 9
// Define Slave answer size
#define ANSWERSIZE 5

#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_FREQ 120

bool DEBUG = 0;
byte radioVals[8];
byte LVert{127},  LHor{127},  RVert{127},  RHor{127};
byte LVert_p{127},  LHor_p{127},  RVert_p{127},  RHor_p{127};

byte LbtnState = 0, RbtnState = 0, LbtnState_p = 0, RbtnState_p = 0;

byte potL, potR, potL_p, potR_p;

int SterL = 0, SterR = 0;
int SterFOR = 0, SterREV = 0;

//[i]     0        1      2       3         4         5         6     7   //8     9  
//    SIDEBRUSH  BRUSH  BLOWER  LIGHTS  CurrMOTOR  BATTERY    GEAR MotLB MotRF MotRB 
byte LCD_DATA[8] = {0,0,0,0,0,0,0,0}; // 12-bit -> 4096 -> 8bit 256 -> coef: 16 

// PWM CH    0        1           2           3
uint16_t BLOWER{0}, BRUSH{0}, SIDEBRUSH{0}, LIGHTS{0};

// MAKS wartosci PWM / 4095
const uint16_t SBRUSH_MAXPWM = 2048; // 50% MAKS
const uint16_t BRUSH_MAXPWM  = 1020; // 25% MAKS
const uint16_t BLOWER_MAXPWM = 4020; // 98% MAKS

byte selectedMotor = 0; // 0-all, 1-sidebrush, 2-brush, 3-blower
//enum MOTOR_TYPE { blower=0, brush=1, sidebrush=2 };
//MOTOR_TYPE cMotor = 0;

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

  //            AForw ARev|BForw BRev
  Serial.println("Start ENGINE");
  motors.motBegin(4,   5,   6,    7);
  pinMode(17, INPUT); // A3
  Serial.println("SETUP END");
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

void loop() {

  if (radio.available()) 
    {
      radio.read(&radioVals, sizeof(radioVals));
      // Analogi - ster.porusz.  BAZOWE co przychodza z pilota
      LVert = radioVals[0];   // 132
      LHor  = radioVals[1];   // 126/127
      RVert = radioVals[2];   // 128
      RHor  = radioVals[3];   // 129
      // Buttony i poty
      LbtnState = radioVals[4]; // OFF/ON
      RbtnState = radioVals[5]; // OFF/ON
      potL = radioVals[6];      // plynne 0-255 POT L
      potR = radioVals[7];      // plynne 0-255 POT R

      motors.drive(LVert, RHor);
      
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
    { calcBat(); sendLCD(); }
    
    // Zmiana LEWA galka HORYZONTALNA
    /*
    if (LHor > 220 && RVdel.del(40))
    {
      if (LIGHT_POWER<3900) LIGHT_POWER+=100;
      pwm.setPWM(3, 0, LIGHT_POWER);
    }
    if (LHor < 35 && RVdel.del(40))
    {
      if (LIGHT_POWER>0) LIGHT_POWER-=100;
      if (LIGHT_POWER < 0) LIGHT_POWER = 0;
      pwm.setPWM(3, 0, LIGHT_POWER);
    }
    */

    // Zmiana PRAWA galka VERTYKALNA - - teraz PRAWY POT
    /*
    if (RVert > 220 && RVdel.del(150))
    {
      if (POWER<3000) POWER+=40;
      pwm.setPWM(0, 0, POWER);
      pwm.setPWM(1, 0, POWER);
      pwm.setPWM(2, 0, POWER);
    }
    if (RVert < 35 && RVdel.del(150))
    {
      if (POWER>0) POWER-=40;
      if (POWER < 0) POWER = 0;
      pwm.setPWM(0, 0, POWER);
      pwm.setPWM(1, 0, POWER);
      pwm.setPWM(2, 0, POWER);
    }
    */
    
} // loop END
