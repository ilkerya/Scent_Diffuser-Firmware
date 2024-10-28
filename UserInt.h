char Dispbuffer[22];  // make sure this is large enough for the largest string it must hold
char* CopyFlashToRam(const char* p){
  //length.Dispbuffer; 
    for (uint8_t i = 0; i < 22; i++) {
      Dispbuffer[i] = 0;
    }
    for (uint8_t i = 0; i < strlen_P(p); i++) {
       char c = pgm_read_byte_near(p + i);
      Dispbuffer[i] = c;
      //Serial.print(c);
    }
   //  Serial.print(F("Dispbuffer : ")); Serial.println(Dispbuffer);
    return Dispbuffer;
}
void UpdateDisplayBuffer(void){  

   // Display_Line1 = " date  ";
   // Display_Line1 +=  " time ";
    Display_Line1 =  "VOC ";
    Display_Line1 += String(Multi_Gas_1.VOC);
    
   // Display_Line1 =  String(Key.Adc); 

    String str;


    //  str = CopyFlashToRam(MESSAGE_PC_MODE1);
   //   Display_Line2 = "2";

    Display_Line3 =  "Bat: " + String(Battery.Volt) + " mV  ";
    Display_Line3 += String(Battery.Adc);// GPIO13

    Display_Line4 = "SW1 " +  String(SW1_In); 
    Display_Line4 += "  SW2 " +  String(SW2_In); 
//display.println(F("DATALOG"));
    Display_Line5 = "Dac1 " +  String(DAC_Val_1);
    Display_Line5 += "  Dac2 " +  String(DAC_Val_2);
      //F("DATALOG"));
      //Display_Line3 = str;    
    /*
    Display_Line4 = "4";   
    Display_Line5 = "5";   
    Display_Line6 = "6";      
    Display_Line7 = "7";   
    Display_Line8 = "8"; 
    */
}

 


#define MAXNOCHAR 4
void UpdateProperLine(uint8_t Index, uint8_t Line){
    #define TEMPERATURE_DEGREE_ASCII 247  // FOR OLED DISPLAY https://en.wikipedia.org/wiki/Code_page_437
    String CelsiusDegree = "";  
    CelsiusDegree = (char)TEMPERATURE_DEGREE_ASCII;
    CelsiusDegree +=  "C";
    
    String str = String(Index)+ ".";    
    switch(Index){
      case DISPROLL_LINE0: 
            str = "";//// show nothing                         
      break; 
        case DISPROLL_LINE1:  


      break;        
      case DISPROLL_LINE2:          

     break;
     case DISPROLL_LINE3:


     break;
     case DISPROLL_LINE4:   
                 
     break;
     case DISPROLL_LINE5:   
        //str += Display_LineTry;
        
     break;      
     case DISPROLL_LINE6:       
          

             
                               
     break; 
     case DISPROLL_LINE7:
           
     break;  

      case DISPROLL_LINE9:       
  
      break;                
      default: str = F("default");
      break; 
    }
    uint16_t Length = str.length();
    if(Length > MAX_DISPLAY_CHAR){ // 34 > 21 //Limit the string to display size
      str.remove(MAX_DISPLAY_CHAR, (Length - MAX_DISPLAY_CHAR));// 21, (34-21) 13
      //str =  String(Index) +  "..." + String(Line);
      //str +=  F(".error");
    }
    if(Length < MAX_DISPLAY_CHAR){// Fill the string to reach full 
      for(uint16_t i = 0; i < (MAX_DISPLAY_CHAR - Length);i++){
        str.setCharAt(Length+i, 'x');
      }    
    }  
}
