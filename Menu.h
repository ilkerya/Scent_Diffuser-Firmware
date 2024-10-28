

void DisplayWakeUp(void) {
  if ((Display.OLED_Timer == 0) && (Display.OLED_Init == OFF)) {
    Display.OLED_Init = ON;
  }//else if(OLED_Timer)OLED_Timer += 10; // add additional time every time any key released
}
 
void  DispExtTimeout(void) {
  if (Display.OLED_Timer <= KEYDISP_TIMER) Display.OLED_Timer = KEYDISP_TIMER;
}
void UpdateInfoQue(uint8_t UpDown){
    #define MAXLINE 9
    #define MINLINE 1
   if (UpDown == DOWNROLL){    // down menu
     DispRollIndex[3] = DispRollIndex[2];
     DispRollIndex[2] = DispRollIndex[1];
     DispRollIndex[1] = DispRollIndex[0];
     DispRollIndex[0]++;
     if (DispRollIndex[0]  > MAXLINE) DispRollIndex[0] = MINLINE;
  } 
  if (UpDown == UPROLL){   
     DispRollIndex[0] = DispRollIndex[1];
     DispRollIndex[1] = DispRollIndex[2];
     DispRollIndex[2] = DispRollIndex[3];
     DispRollIndex[3]--;  
     if (DispRollIndex[3]  < MINLINE) DispRollIndex[3] = MAXLINE; 
   } 
}
void UpdateDispRoll(uint8_t UpDown){
    if(Display.SensorRollTimer){
      Display.SensorRollTimer--;
      return;
    }
    UpdateInfoQue(UpDown);
}
void KeySensonsorRoll(uint8_t UpDown){
      Display.SensorRollTimer = 30; // 2sec x 30 = 60 sec
      UpdateInfoQue(UpDown);      
}
void DispEnable(bool Enable, uint8_t Timer) {
  if (Enable == ON) {
    Display.SleepEnable = ON; //go sleep
    Display.OLED_Timer = Timer;
  }
  else   Display.SleepEnable = OFF;    // no sleep
}


