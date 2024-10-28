/*
 Repeat timer example

 This example shows how to use hardware timer in ESP32. The timer calls onTimer
 function every second. The timer can be stopped with button attached to PIN 0
 (IO0).

 This example code is in the public domain.
 */

// Stop button is attached to PIN 0 (IO0)

#include <Wire.h>

  #include <Adafruit_Sensor.h>
  #include "Adafruit_BME680.h"
  #define SEALEVELPRESSURE_HPA (1013.25)
  Adafruit_BME680 bme; // I2C

//#include <RTClib.h>
#include <Adafruit_Si7021.h>
#include <Adafruit_SSD1306.h>



 #include "M_Gas.h"
GAS_GMXXX<TwoWire> gas;
#include  "Defs.h"
#include "Variables.h"
#include "DAQ.h"
#include "Functions.h"
#include  "UserInt.h"
#include  "Display.h"
#include  "Menu.h"

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
 //pinMode(BTN_STOP_ALARM, INPUT);
  gas.begin(Wire, 0x08); // use the hardware I2C

  Interrupt_Set();
   DisplaySetPowerIO();
    Init_IO();
    DisplayInit();   
//   pinMode(SWITCH_2, OUTPUT);
 // digitalWrite(SWITCH_2,HIGH); 
   //analogWrite(ledPin, dutyCycle);
}


void loop() {
  // If Timer has fired
  if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE){
    uint32_t isrCount = 0, isrTime = 0;
    // Read the interrupt count and time
    portENTER_CRITICAL(&timerMux);
    isrCount = isrCounter;
    isrTime = lastIsrAt;
    portEXIT_CRITICAL(&timerMux);
    // Print it
  /*
    Serial.print("onTimer no. ");
    Serial.print(isrCount);
    Serial.print(" at ");
    Serial.print(isrTime);
    Serial.println(" ms");
*/
       Gas += 0.1;
    if(Gas > 90)Gas = 70;

    DAC_Val_1++;
    DAC_Val_2--;

        Bosch_BME688_1.Gas = Gas+1;
        Bosch_BME688_2.Gas = Gas+2;
        Bosch_BME688_3.Gas = Gas;     
        Bosch_BME688_4.Gas = Gas-1;    
        Gas_Convert();  
        Battery_Volt();   
        DAC_Write();
        SW_Read();


   // DAQ_Send_Data(LOOP_BASED); 
  

   DisplayScreen();

   // Scan_I2C();
   //Gas_Convert();

  }
  // If button is pressed
  /*
  if (digitalRead(BTN_STOP_ALARM) == LOW) {
    // If timer is still running
    if (timer) {
      // Stop and free timer
      timerEnd(timer);
      timer = NULL;
    }
  }
  */
}


