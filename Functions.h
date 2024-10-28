hw_timer_t * timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile uint32_t isrCounter = 0;
volatile uint32_t lastIsrAt = 0;
void ARDUINO_ISR_ATTR onTimer(){
  // Increment the counter and set the time of ISR
  portENTER_CRITICAL_ISR(&timerMux);
  isrCounter = isrCounter + 1;
  lastIsrAt = millis();
  portEXIT_CRITICAL_ISR(&timerMux);
  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
  // It is safe to use digitalRead/Write here if you want to toggle an output
           LED_STATE= !LED_STATE;  				//toggle logic
       digitalWrite(LED_PIN, LED_STATE);    //Toggle LED
}
void Interrupt_Set(void){
  // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();
#define TIME_USEC 100000 // 50 msec resoltion
  // Set timer frequency to 1Mhz resolution
  timer = timerBegin(1000000);

  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer);

  // Set alarm to call onTimer function every second (value in microseconds).
  // Repeat the alarm (third parameter) with unlimited count = 0 (fourth parameter).
  timerAlarm(timer, TIME_USEC, true, 0);// 100000 usec => 1000 msec => 1 sec

 // pinMode(BTN_STOP_ALARM, INPUT);
}
void  Init_IO(void){
  pinMode(SWITCH_1, INPUT_PULLUP );
  pinMode(SWITCH_2, INPUT_PULLUP );

  pinMode(I2C_1_PWR, OUTPUT);
  pinMode(I2C_2_PWR, OUTPUT);   

  pinMode(BOOST_EN, OUTPUT);  

  pinMode(BOOST_DAC_1, OUTPUT);  
  pinMode(BOOST_DAC_2, OUTPUT);  
 
}
void SW_Read(void){
  SW1_In = digitalRead(SWITCH_1); 
  SW2_In = digitalRead(SWITCH_2); 
}




void DAC_Write(void){

   // https://www.electronicwings.com/esp32/dac-digital-to-analog-converter-esp32
    dacWrite(BOOST_DAC_1, DAC_Val_1);//0-255 0-3V3 A1_DAC1_IO25 
    dacWrite(BOOST_DAC_2, DAC_Val_2);//0-255 0-3V3 A1_DAC1_IO25 
   // dacWrite(26, 100);// A0_DAC2_IO26
   //dacDisable(25);
}
void Battery_Volt(void){
 // uint16_t Battery_Volt; 
  Battery.Adc = analogRead(BATTERY_ADC);

 // uint32_t temp = 3300;
  Battery.Volt_32 = Battery.Adc * 3300;
 // Battery.Volt_32 /= 4096;
  Battery.Volt_32 /= 2048;   // 100K/100K voltage Divider
  Battery.Volt = (uint16_t)Battery.Volt_32;

}
void Gas_Convert(){
    Multi_Gas_1.VOC = gas.measure_VOC(); 
   // Serial.print(F(" VOC:")); Serial.print(Multi_Gas_1.VOC); Serial.print(F(" /"));
   // Serial.print(gas.calcVol(Multi_Gas_1.VOC)); Serial.print("V  ");
    }
