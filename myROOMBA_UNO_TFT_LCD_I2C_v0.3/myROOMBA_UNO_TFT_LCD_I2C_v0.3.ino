#define LCD_CS A3 // Chip Select goes to Analog 3
#define LCD_CD A2 // Command/Data goes to Analog 2
#define LCD_WR A1 // LCD Write goes to Analog 1
#define LCD_RD A0 // LCD Read goes to Analog 0
#define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin

#include <SPI.h>          // f.k. for Arduino-1.5.2
#include "Adafruit_GFX.h"// Hardware-specific library
#include <MCUFRIEND_kbv.h>
MCUFRIEND_kbv tft;

// Assign human-readable names to some common 16-bit color values:
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

#ifndef min
#define min(a, b) (((a) < (b)) ? (a) : (b))
#endif

#include <Wire.h>

#define SLAVE_ADDR 9
#define ANSWERSIZE 5 // po nic narazie
String answer = "Romba"; // response tez po nic narazie

word hsv2hex(byte h, byte s, byte v)
{
    double  H, S, V, P, Q, T, RC,GC,BC, fract;
    H = 1.4117647058823529411764705882353*h; // mapuj 0-360 -> 0-255
    S = 0.0039215686274509803921568627451*s; //       0-1   -> 0-255
    V = 0.0039215686274509803921568627451*v; //       0-1   -> 0-255
    byte  bR, bG, bB;
    (H == 360.)?(H = 0.):(H /= 60.);
    fract = H - floor(H);
    P = V*(1. - S); Q = V*(1. - S*fract); T = V*(1. - S*(1. - fract));
    if(0. <= H && H < 1.){RC = V; GC = T; BC = P;}
    else if (1. <= H && H < 2.){RC = Q; GC = V; BC = P;}
    else if (2. <= H && H < 3.){RC = P; GC = V; BC = T;}
    else if (3. <= H && H < 4.){RC = P; GC = Q; BC = V;}
    else if (4. <= H && H < 5.){RC = T; GC = P; BC = V;}
    else if (5. <= H && H < 6.){ RC = V; GC = P; BC = Q;}
    else{RC = 0; GC = 0; BC = 0;}
    bR = byte(RC*255); bG = byte(GC*255); bB = byte(BC*255);
    return ( ((bR & 0xF8) << 8) | ((bG & 0xFC) << 3) | (bB >> 3) );
}
#include "gui.h"

void setup() {
  Wire.begin(SLAVE_ADDR);
  Wire.onRequest(requestEvent); // narazie zbedne
  Wire.onReceive(receiveEvent);
  // Setup Serial Monitor 
  Serial.begin(115200);
  Serial.println("I2C TFT na addr: 9");
  uint16_t ID = tft.readID(); //
  Serial.print("ID = 0x");
  Serial.println(ID, HEX);
  if (ID == 0xD3D3) ID = 0x9481; // ID = 0x9329; // force ID
  tft.begin(ID);

  tft.setRotation(1);
  tft.fillScreen(RED);
  tft.setCursor(100, 100);
  tft.setTextColor(WHITE); tft.setTextSize(2);
  String mess = "STARTING ROBOL";
  tft.println(mess);
  delay(800);
  tft.fillScreen(BLACK);
  delay(50);
}
//[i]     0        1      2       3         4         5         6       7(bits)      8       9
//    SIDEBRUSH  BRUSH  BLOWER  LIGHTS  CurrMOTOR  BATTERY    GEAR    BLCK/InvRev  SonarL  SonarR
byte LCD_DATA[10] =   {0,0,0,0,0,0,3,0,0,0};
byte LCD_DATA_p[10] = {0,0,0,0,0,0,3,0,0,0};

// STALE MAX PWM z BAZY NANO -> trzeba zupdatowac jak zmienie na NANO
const uint16_t SBRUSH_MAXPWM = 2048; // 50% MAKS
const uint16_t BRUSH_MAXPWM  = 1020; // 25% MAKS
const uint16_t BLOWER_MAXPWM = 4020; // 98% MAKS

int iter = 0;
float BATV, BATVp;
int bPc, bPc_p;
bool ONCE = true;
bool BatIncomFLAG = false;

bool Rblck{0}, Rblck_p{0}, Lblck{0}, Lblck_p{0}, RevInv{0}, RevInv_p{0}, AutoAI{0}, AutoAI_p{0};

void receiveEvent() {
  BatIncomFLAG = true;
  iter=0;
  while (0 < Wire.available()) {
    LCD_DATA[iter] = Wire.read();
    iter++;
    if (iter > 10) break;
  }
} // DATA INCOMMING
 
void requestEvent() {
  // Setup byte variable in the correct size
  byte response[ANSWERSIZE];
  // Format answer as array
  for (byte i=0;i<ANSWERSIZE;i++) {
    response[i] = (byte)answer.charAt(i);
  }
  // Send response back to Master
  Wire.write(response,sizeof(response));
  // Print to Serial Monitor
  //Serial.println("Request event");
}


/*-- GUI ELEMENTS ------*/
valBarDisp lightBar;
gearbox gBox(60,45,10);
valBarDispHor BRUSH_BAR, SBRUSH_BAR, BLOWER_BAR;
sFrame cmFrame(2);
valBarDisp BATT_PC;
/*-- GUI ELEMENTS END --*/

uint32_t BATSUM = 0;
byte BATMED = 0;
int BATITER = 0;

int LEFF = 15;
String mess = "";

void showImage565(int x, int y, int w, int h, const uint8_t img[])
{
    tft.setAddrWindow(x, y, x + w - 1, y + h - 1);
    tft.pushColors(img, w * h, 1);
}

bool OORclrFlagR = false, OORclrFlagL = false;
void loop() {

  if (ONCE)
  {
    tft.fillScreen(BLACK);
    gBox.draw(250, 160);
    gBox.setGear(3);
    BATT_PC.drawLikeBat(0, 0, 100, 80, 10, 155, 20);
    tft.setTextColor(YELLOW); tft.setTextSize(2);
    tft.setCursor(156, 170);
    tft.print("LIGHTS");
    showImage565(30, 210, 20, 20, SBRSHimg20x20);
    showImage565(70, 210, 20, 20, BRSHimg20x20);
    showImage565(110, 210, 20, 20, BLOWimg20x20);
  }

  if (LCD_DATA_p[0] != LCD_DATA[0] || ONCE==true) // SIDEBRUSH
  {
    /*
    tft.setTextColor(BLACK); tft.setTextSize(1);
    tft.setCursor(LEFF, 40);
    mess = "SBR: "+String(LCD_DATA_p[0]);
    tft.print(mess);
    tft.setTextColor(YELLOW); tft.setTextSize(1);
    tft.setCursor(LEFF, 40);
    mess = "SBR: "+String(LCD_DATA[0]);
    tft.print(mess);
    */
    // BAR
    SBRUSH_BAR.draw(LCD_DATA[0], 0, 255, 30, 120, 20, 80);
    
    LCD_DATA_p[0] = LCD_DATA[0];
  }
  if (LCD_DATA_p[1] != LCD_DATA[1] || ONCE==true) // BRUSH
  {
    // BAR
    BRUSH_BAR.draw(LCD_DATA[1], 0, 255, 70, 120, 20, 80);
    LCD_DATA_p[1] = LCD_DATA[1];
  }
  if (LCD_DATA_p[2] != LCD_DATA[2] || ONCE==true) // BLOWER
  {
    BLOWER_BAR.draw(LCD_DATA[2], 0, 255, 110, 120, 20, 80);
    LCD_DATA_p[2] = LCD_DATA[2];
  }
  if (LCD_DATA_p[3] != LCD_DATA[3] || ONCE==true) // LIGHTS
  {
    lightBar.draw(LCD_DATA[3], 0, 255, 155, 190, 70, 30);
    LCD_DATA_p[3] = LCD_DATA[3];
  }
  if (LCD_DATA_p[4] != LCD_DATA[4] || ONCE==true) // CurrMOTOR
  {
    /*
    tft.setTextColor(BLACK); tft.setTextSize(1);
    tft.setCursor(LEFF, 110);
    mess = "CMT: "+String(LCD_DATA_p[4]);
    tft.print(mess);
    tft.setTextColor(YELLOW); tft.setTextSize(1);
    tft.setCursor(LEFF, 110);
    mess = "CMT: "+String(LCD_DATA[4]);
    tft.print(mess);
    */
    cmFrame.clr(20, 110, 120, 125);
    cmFrame.clr(20, 110, 40, 125);
    cmFrame.clr(60, 110, 40, 125);
    cmFrame.clr(100, 110, 40, 125);
    // 0-WSZYSTKIE, 1-SIDEBRUSH, 2-BRUSH, 3-BLOWER ~ obszar x30, y120, w130, h230
    switch(LCD_DATA[4])
    {
      case 0: // ALL
      {
        cmFrame.draw(20, 110, 120, 125, TFT_WHITE);
        break;
      }
      case 1: // SBRUSH
      {
        cmFrame.draw(20, 110, 40, 125, TFT_RED);
        break;
      }
      case 2: // BRUSH
      {
        cmFrame.draw(60, 110, 40, 125, TFT_RED);
        break;
      }
      case 3: // BLOWER
      {
        cmFrame.draw(100, 110, 40, 125, TFT_RED);
        break;
      }
      default: break;
    }
    
    LCD_DATA_p[4] = LCD_DATA[4];
  }
  if ( BatIncomFLAG == true ) // BATTERY
  {
    BatIncomFLAG = false;
     /* MIN 12.8    16.8 MAX
    ArdV    1.98    2.6   xCoef 6.46
    ArdAna  405.5   532.5 
    byte    101,375 133,125 
    BatV = 5/1024 * ArdAna * coef  
    */
    if (BATITER < 15)
    {
      BATSUM += LCD_DATA[5];
      BATITER++;
    }
    else 
    {
      BATITER=0;
      int BSS = static_cast<int>(round(BATSUM/15));
      BATMED = static_cast<byte>(BSS);
      BATSUM=0;
      
      if (BATMED < 103)     bPc = 0;
      else if(BATMED > 131) bPc = 100;
      else bPc = map(BATMED, 103, 131, 1, 99);
  
      int original = (BATMED)*4;
      float BATV = 5./1023*original * 6.46; // 6.46 -> coef z dzielnika nap.
    
      tft.setTextColor(BLACK); tft.setTextSize(2);
      tft.setCursor(85, 50);
      mess = "B "+String(bPc_p)+"% "+String(BATVp)+" V";
      tft.print(mess);
      tft.setTextColor(YELLOW); tft.setTextSize(2);
      tft.setCursor(85, 50);
      mess = "B "+String(bPc)+"% "+String(BATV,1)+" V";
      tft.print(mess);

      //BAR BATTERY %
      BATT_PC.drawLikeBat(bPc, 0, 100, 80, 10, 155, 20);
      
      BATVp = BATV;
      bPc_p = bPc;
    }
  } // BAT
  
  if (LCD_DATA_p[6] != LCD_DATA[6]) // GEAR
  {
    gBox.setGear(LCD_DATA[6]);
    LCD_DATA_p[6] = LCD_DATA[6];
  }

  if (LCD_DATA_p[7] != LCD_DATA[7]) // l/r blocking - invert reverse directions - AUTO AI
  {
    Rblck = LCD_DATA[7] & 0x1;
    Lblck = (LCD_DATA[7] & 0x2)>>1;
    RevInv = (LCD_DATA[7] & 0x4)>>2;
    AutoAI = (LCD_DATA[7] & 0x8)>>3;
    if (Rblck==true)
    { tft.setTextColor(TFT_RED); tft.setTextSize(2); tft.setCursor(270, 10); tft.print("BLC!"); }
    else 
    { tft.setTextColor(TFT_BLACK); tft.setTextSize(2); tft.setCursor(270, 10); tft.print("BLC!"); }
    if (Lblck==true)
    { tft.setTextColor(TFT_RED); tft.setTextSize(2); tft.setCursor(10, 10); tft.print("BLC!"); }
    else 
    { tft.setTextColor(TFT_BLACK); tft.setTextSize(2); tft.setCursor(10, 10); tft.print("BLC!"); }

    if (RevInv==true)
    { tft.setTextColor(TFT_RED); tft.setTextSize(2); tft.setCursor(90, 80); tft.print("Rev-Invrt"); }
    else 
    { tft.setTextColor(TFT_BLACK); tft.setTextSize(2); tft.setCursor(90, 80); tft.print("Rev-Invrt!"); }
    LCD_DATA_p[7] = LCD_DATA[7];

    if (AutoAI==true)
    { tft.setTextColor(TFT_RED); tft.setTextSize(2); tft.setCursor(160, 110); tft.print("AutoAI!"); }
    else 
    { tft.setTextColor(TFT_BLACK); tft.setTextSize(2); tft.setCursor(160, 110); tft.print("AutoAI!"); }
    LCD_DATA_p[7] = LCD_DATA[7];
  }

  // SONARY L/R
  if ((LCD_DATA_p[8] != LCD_DATA[8]) && (LCD_DATA[8] > 2 && LCD_DATA[8] < 30)) // Sonar L
  {
      tft.setTextColor(TFT_BLACK); tft.setTextSize(2); tft.setCursor(10, 29); tft.print(LCD_DATA_p[8]);
      if (LCD_DATA[8] > 20) { tft.setTextColor(TFT_WHITE); }
      else if (LCD_DATA[8] <= 20 && LCD_DATA[8] >= 10) { tft.setTextColor(TFT_YELLOW); }
      else { tft.setTextColor(TFT_RED); }
      tft.setTextSize(2); tft.setCursor(10, 29); tft.print(LCD_DATA[8]);
      LCD_DATA_p[8] = LCD_DATA[8];  
      OORclrFlagL = false;  
  }
  else if (((LCD_DATA_p[8] != LCD_DATA[8]) && (LCD_DATA[8] <= 2 || LCD_DATA[8] >= 30)) && OORclrFlagL==false)
  {
    tft.setTextColor(TFT_BLACK); tft.setTextSize(2); tft.setCursor(10, 29); tft.print(LCD_DATA_p[8]);
    OORclrFlagL = true;
  }
  
  if ((LCD_DATA_p[9] != LCD_DATA[9]) && (LCD_DATA[9] > 2 && LCD_DATA[9] < 30)) // Sonar R
  {
      tft.setTextColor(TFT_BLACK); tft.setTextSize(2); tft.setCursor(270, 29); tft.print(LCD_DATA_p[9]);
      if (LCD_DATA[9] > 20) { tft.setTextColor(TFT_WHITE); }
      else if (LCD_DATA[9] <= 20 && LCD_DATA[9] >= 10) { tft.setTextColor(TFT_YELLOW); }
      else { tft.setTextColor(TFT_RED); }
      tft.setTextSize(2); tft.setCursor(270, 29); tft.print(LCD_DATA[9]);
      LCD_DATA_p[9] = LCD_DATA[9];  
      OORclrFlagR = false;  
  }
  else if (((LCD_DATA_p[9] != LCD_DATA[9]) && (LCD_DATA[9] <= 2 || LCD_DATA[9] >= 30)) && OORclrFlagR==false)
  {
    tft.setTextColor(TFT_BLACK); tft.setTextSize(2); tft.setCursor(270, 29); tft.print(LCD_DATA_p[9]);
    OORclrFlagR = true;
  }
  //OORclrFlagR

  ONCE = false;
}
