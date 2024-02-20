#ifndef AIBLOCK_H
#define AIBLOCK_H

// Vars
bool AItrig = false;
uint32_t AIstCounter = 0;
bool isAIinControll = false;
Kronos AIactivatingDel;
byte OBSTACLEDIS = 0;
Kronos AIatractor;
//Kronos asyncDel;
//int currBlock = -1;

// Funcs
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
    rearLed.writeG(0);
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
    rearLed.writeG(255);
    delay(500);
  }
}

void AI_drive()
{
  OBSTACLEDIS = readUltrasonicDistance(LSonarPin,LSonarPin);
  delay(50);
  if ( (OBSTACLEDIS < 3 || OBSTACLEDIS > 25 && (LisBlck==false && RisBlck==false)) )
  {
    if (AIatractor.del(random(4000, 13000))) { motors.drive(132, random(50,250)); delay(random(200,600)); } // rand dla drobnych zmian
    motors.drive(250, 129);
  }
  else if ((OBSTACLEDIS >=3 && OBSTACLEDIS <= 25) )
  {
    motors.drive(132, random(50,250));
    delay(random(300,900));
    motors.drive(132, 129);
    OBSTACLEDIS = readUltrasonicDistance(LSonarPin,LSonarPin);
    delay(300);
  }
  else if ((LisBlck==true || RisBlck==true)  )
  {
    if (LisBlck==true && RisBlck==false  )
    {
      motors.drive(50, 129);
      delay(random(400,700));
      motors.drive(132, random(180,250));
      delay(random(400,900));
    }
    else if ((LisBlck==false && RisBlck==true)  )
    {
      motors.drive(50, 129);
      delay(random(400,700));
      motors.drive(random(180,250), 127);
      delay(random(400,900));
    }
    else
    {
      motors.drive(50, 129);
      delay(random(400,700));
      motors.drive(random(50,200), 127);
      delay(random(400,900));
    }
    
  }
  else motors.drive(132, 129); // STOP jezeli nie wiadomo co sie dzieje
}

#endif
