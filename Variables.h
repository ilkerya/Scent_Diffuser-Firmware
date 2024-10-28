
#define BTN_STOP_ALARM    0


uint8_t SW1_In;
uint8_t SW2_In;

uint8_t DAC_Val_1 = 127;
uint8_t DAC_Val_2 = 127;

struct
{
  uint16_t Volt;    // 31
  uint16_t Adc;   // 35
  int32_t Volt_32;
}Battery;

//int LED_STATE=LOW;
uint8_t LED_STATE;;
float Gas = 80.0;

  struct Sensor_BME688_Bosch
{
  float Temperature = 0;
  float Humidity = 0;  
  float Pressure = 0;
  float Gas = 0; 
  uint8_t Exists = 0; 
};

  Sensor_BME688_Bosch Bosch_BME688_1;
  Sensor_BME688_Bosch Bosch_BME688_2;
  Sensor_BME688_Bosch Bosch_BME688_3;
  Sensor_BME688_Bosch Bosch_BME688_4;


struct TaskOrg
{
  uint8_t  IntTimer_10 = 0;
  uint8_t  IntTimer_100 = 0;
  uint16_t  IntTimer_250 = 0;
  uint16_t  IntTimer_500 = 0;
  uint16_t  IntTimer_1_Sec = 0;
  uint16_t  IntTimer_2_Sec = 0;
  uint16_t  IntTimer_5_Sec = 0;
  uint16_t  IntTimer_10_Sec = 0;
  uint16_t  IntTimer_20_Sec = 0;
  uint16_t  IntTimer_60_Sec = 0;  
  bool Task_SameInt =0;
  bool Task_10msec =0;
  bool Task_100msec =0;
  bool Task_250msec =0;
  bool Task_500msec =0;
  bool Task_1Sec =0;
  bool Task_2Sec =0;
  bool Task_5Sec =0;
  bool Task_10Sec =0;
  bool Task_20Sec =0;
  bool Task_60Sec =0;
};
 TaskOrg Loop;

#define TASK_500MSEC  1
#define TASK_1SEC 2
#define TASK_2SEC 4
#define TASK_5SEC 8
#define TASK_10SEC 16
#define TASK_20SEC 32
#define TASK_60SEC 64

#define ON 1
#define OFF 0

struct
{
  float Humidity_OnBoard;
  float Temperature_OnBoard; // 27  
  float Humidity_Ch1;
  float Temperature_Ch1; // 27
  float Humidity_Ch2;
  float Temperature_Ch2; // 27
  float Humidity_Ch3;
  float Temperature_Ch3; // 27  
  float Current;
  float Voltage;
  float PowerFactor; 
  float ActivePower;
  float Frequency;
  float Pressure; //  
  float TemperatureBMP; //  
  float Altitude; //
  float PM25=0; //
  float PM10=0; //   
  float WindRaw;   // 35        
  uint16_t WindMPH;    // 31
  uint16_t WindTemp;   // 35
  uint16_t Luminosity;  
  int32_t DAQ_Temperature;
  int32_t DAQ_Humidity; 
}Values;

static const unsigned char PROGMEM logo16_glcd_bmp[] =
{ 0b00000000, 0b11000000,
  0b00000001, 0b11000000,
  0b00000001, 0b11000000,
  0b00000011, 0b11100000,
  0b11110011, 0b11100000,
  0b11111110, 0b11111000,
  0b01111110, 0b11111111,
  0b00110011, 0b10011111,
  0b00011111, 0b11111100,
  0b00001101, 0b01110000,
  0b00011011, 0b10100000,
  0b00111111, 0b11100000,
  0b00111111, 0b11110000,
  0b01111100, 0b11110000,
  0b01110000, 0b01110000,
  0b00000000, 0b00110000 };
  /*
//char Disp_MENU1[] =    {'L','O','G',' ','S','T','A','R','T',' ','&',' ','S','T','O','P',' ','M','E','N','U','\0'};
static const char Disp_MENU8_SUB1[] PROGMEM =      " Portable -> SD Card ";
static const char Disp_MENU8_SUB2[] PROGMEM =      "  PC  -> 115200 bps  ";
static const char Disp_MENU8_SUB3[] PROGMEM =      " Log Mode Updated !  ";
static const char Disp_MENU2_SUB[] PROGMEM = "Enter -> ";  //9
  static const char LOG_1MSEC[]   PROGMEM = "  1 mS"; //12
  static const char LOG_5MSEC[]   PROGMEM = "  5 mS"; //12  
*/
String Display_Line1 ="Display.........Line1"; 
String Display_Line2 ="Display........Line2."; 
String Display_Line3 ="Display.......Line3.."; 
String Display_Line4 ="Display......Line4..."; 
String Display_Line5 ="Display.....Line5....";
String Display_Line6 ="Display....Line6.....";
String Display_Line7 ="Display...Line7......";
String Display_Line8 ="Display..Line8.......";

uint8_t MainMenu =0;
uint8_t DispRollIndex[4] = {1,0,0,0};

struct
{
  bool RTC_Update=0; 
  bool OLED_Init = 0 ; 
  uint8_t MenuTimeout=0; 
  uint8_t Flash=0; 
  uint16_t OLED_Timer = 0; 
  bool InitDelay = 0;
  bool SleepEnable = 0;
  uint8_t ValueTimer = 0; 
  bool ExpSensOnb =0;  
  bool ExpSens1 =0; 
  bool ExpSens2 =0;
  bool ExpSens3 =0;
  uint8_t SensorRollTimer = 0;
  uint8_t ReInit_Timer = 2;
  bool ReInit_Enable = OFF;
}Display;


