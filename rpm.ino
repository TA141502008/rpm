/*
 * Project        : Firmware Prototype for Remote Monitoring Platform
 *                  Sistem Monitoring Kualitas Air Tambak Udang Vaname
 * Version        : 0.17
 * Date Created   : July 7, 2015
 * Date Modified  : December 7, 2015
 * Author         : Adya Verol Lutfi 
 * Company        : Department of Electrical Engineering
 *                  School of Electrical Engineering and Informatics
 *                  Bandung Institute of Technology (ITB)
 * Summary        : 
 */

#include <OneWire.h> 
#include <LiquidCrystal.h>
#include <VirtualWire.h>
#include <string.h>
//#include <NewPing.h>
//--------------------------------------- Pin Configuration --------------------------------------------
const int DS18S20_Pin = 30;           // DS18S20 Signal pin on digital pin 30
const int Button_Pin = 22;            // LCD pushbutton pin on digital pin 22
const int control_BJT_temperature = 34; // Temperature probe BJT controller/base on digital pin 34
const int control_BJT_pH = 36;      // pH probe BJT controller/base on digital pin 36
const int control_BJT_salinity = 38;  // Salinity probe BJT controller/base on digital pin 38
const int control_BJT_DO = 40;  // Salinity probe BJT controller/base on digital pin 38
const int LCDbacklight_Pin = 9;       // LCD Backlight controller using Enable pin connected to digital pin 9
#define Sensor_pH_Pin 0               // pH meter Analog output to Arduino Analog Input 0
#define Sensor_turbidity_Pin 8        // Output turbidity sensor to Analog Input 8    
#define Sensor_salinity_Pin 9         // Output salinity sensor to Analog Input 9   

//======================================= Constant Variables =============================================
LiquidCrystal lcd(12, 8, 5, 4, 3, 2);   // LCD pin initialization
OneWire ds(DS18S20_Pin);                // Onewire library used by DS1820 on digital pin 30
int Sensor_interval = 20;
int Sensor_active_interval_1 = 15;
int Sensor_active_interval_2 = 10;

//======================================= Constant Variables End =============================================

//====================================== Changing Variables ============================================

//---------------------------------------- MISC Variable ------------------------------------------------
unsigned long Detik_Now = 0;        // Variable used to show how many seconds passed since the last reset
unsigned long Detik_Previous = 0;   // Used for rollover prevention   
float temperature = 0;        // Temperature measurement result
float temperature_2 = 0;        // Temperature measurement result
float SalinityVoltage;        // Salinity Measurement Result in voltage
float Salinity_R;         // Salinity measurement result in resistance
float Salinity_Measurement;     // salinity measurement resutl in g/L
float Salinity_Measurement_old;     // salinity measurement resutl in g/L
float temperature_measurement = 0;  // temperature measurement result used to compensate DO measurement
int temperature_loop_count = 0;
int salinity_loop_count = 0;

//------------------------------------ LCD and Button Variable ------------------------------------------
int Button_Push_Counter = 0;    // Counter for the number of button presses
int Button_State_22 = 0;        // Current state of the button at Pin 22
int Active_Display = 1;         // Variable for current active display
int Button_delay = 0;           // To measure how long the button is pressed
int LCD_state = 0;              // Current State of the LCD
int last_button_pressed = 0;

//--------------------------------------- Transmitter Variable ------------------------------------------
float dummy_salinitas = 56.78;

char Packet_Temperatur[6];      // Packet part that contains temperature measurement result
char Packet_Temperatur_2[6];      // Packet part that contains temperature measurement result
char Packet_pH[6];              // Packet part that contains pH measurement result
char Packet_pH_2[6];            // Packet part that contains pH measurement result with fixed length of string
char Packet_Kekeruhan[6];       // Packet part that contains turbidity measurement result
char Packet_Kekeruhan_2[6];     // Packet part that contains turbidity measurement result with fixed length of string
char Packet_DO[6];              // Packet part that contains DO measurement result
char Packet_DO_2[6];            // Packet part that contains DO measurement result with fixed length of string
char Packet_Salinitas[6];       // Packet part that contains salinity measurement result
char Packet_Salinitas_2[6];     // Packet part that contains salinity measurement result with fixed length of string
char Packet_Total[40];          // Combined packet part that being sent to the HMI module

int sent_ready_flag = 0;    // Flag used to tell if the receiver is ready to sent another packet to the HMI
int Sent_interval = 120;     // Time interval between every packet transmission, in second

//--------------------------------------------- DO Variable ------------------------------------------
char DO_Temp_Compensation_variable[13];     // Latest temperature measurement result that used for DO compensation
int DO_Temp_Compensation_Counter = 0;       // Count number of loop since the last temperatur compensation for DO probe

String input_DO_string = "";                // String to hold incoming data from the PC
String sensor_DO_string = "";               // String to hold the data from the Atlas Scientific probe
boolean input_DO_string_complete = false;   // Check if received all the data from the PC
boolean sensor_DO_string_complete = false;  // Check if received all the data from the Atlas Scientific probe
float DO_measurement;                       // DO probe measurement result
float DO_measurement_last;                  // DO probe measurement result, before the current result
int DO_loop_count = 0;
int DO_receive_data_count = 0;
//--------------------------------------------- pH Variable ------------------------------------------
unsigned long int Average_Value_pH;       // Store the average value of the pH probe result
int buf_pH[10],temp_pH;                   // Buffer and temporary variable used to sort and averaging pH measurement result
float pH_measurement = 0;
int pH_loop_count = 0;

//---------------------------------------- Turbidity Variable ----------------------------------------
unsigned long int Average_Value_turbidity;  // Store the average value of the turbidity probe result
int buf_turbidity[10],temp_turbidity;       // Buffer and temporary variable used to sort and averaging turbidity measurement result

//-------------------------------------------------- Flag ----------------------------------------------
boolean salinity_read_ready_flag = false;    // Flag to toggle the salinity probe
boolean ph_read_ready_flag = false;        // Flag to toggle the pH probe
boolean temperature_read_ready_flag = false;   // Flag to toggle the temperature probe
boolean DO_read_ready_flag = false;        // Flag to toggle the DO probe
boolean DO_data_flag = false;          // Flag to tell if DO probe ready to sent data to RMP
int DO_detik_count = 0;
int temperature_detik_count = 0;
int pH_detik_count = 0;
int salinity_detik_count = 0;

//====================================== Changing Variables End ============================================

//============================================= SETUP ==================================================
void setup(void) 
{
  analogReference(DEFAULT);
  Serial.begin(9600);                     // Serial communication baud rate set to 9600
  Serial3.begin(9600);                              
  
  input_DO_string.reserve(10);            // String reserve for input command from PC set to 10     
  sensor_DO_string.reserve(30);           // String reserve for DO probe result set to 30
  
  pinMode(Button_Pin, INPUT);             // Initialize the button pin as a input
  pinMode(13, OUTPUT);                    // Inititalize Pin 13 as Output for LED

  pinMode(control_BJT_temperature, OUTPUT); // Initialize pin BJT control to output
  pinMode(control_BJT_pH, OUTPUT);
  //pinMode(control_BJT_salinity, OUTPUT);
  pinMode(control_BJT_DO, OUTPUT);
  
  
  // LCD setup and initialization
  lcd.begin(16, 2);                         // Set up the LCD's number of columns and rows: 
  pinMode(LCDbacklight_Pin, OUTPUT);        // Initialize LCD back light pin as output
  digitalWrite(LCDbacklight_Pin, LOW);     // LCD backlight default is off
  lcd.noDisplay();
  
  // RF transmitter setup and initizalitaion
  vw_set_ptt_inverted(true);              
  vw_set_tx_pin(26);                        // Initialize pin 26 as transmitter output data pin
  vw_setup(4000);                           // Set transmitter data rate to 4 kbps

  DO_detik_count = 2 * Sensor_interval;
  temperature_detik_count = 0.5 * Sensor_interval;
  pH_detik_count = 4.5 * Sensor_interval;
  //salinity_detik_count = 3 * Sensor_interval;
  
  Serial.println("Ready");                  // Test the serial monitor
}
//========================================== SETUP END ==================================================

//===================================== Additional Procedure and Function ================================

 // ---------------------------------------- DO ------------------------------------------ 
void serialEvent3()                                 // If the hardware serial port_3 receives a char
{                                                
  sensor_DO_string = Serial3.readStringUntil(13);   // Read the string until <CR>
  sensor_DO_string_complete = true;                 // Set the flag used to tell if finished receiving string from the serial port 3 --> DO Probe
}

//--------------------------------------- temperature ---------------------------------------
// this algorithm is acquired from dallas semiconductor DS1820 library -- Copyright (c) 2010 bildr community

float getTemp()                                     // Returns the temperature from one DS18S20 in DEG Celsius
{
  byte probe_data[12];
  byte addr[8];
  if ( !ds.search(addr)) {                
      ds.reset_search();                            // No more sensors on chain, reset search
      return -9;                                    
  }
  if ( OneWire::crc8( addr, 7) != addr[7]) {        // 8 bit Dallas Semiconductor Cyclic Redundancy Check
      Serial.println("CRC is not valid!");
      return -9;
  }
  if ( addr[0] != 0x10 && addr[0] != 0x28) {        // Device signature recognization from Onewire Livrary, used for multi probe measurement
      Serial.print("Device is not recognized");
      return -9;
  }
  ds.reset();
  ds.select(addr);
  ds.write(0x44,1);                                 // start conversion, with parasite power on at the end
  byte present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);                                   // Read from the Scratchpad
  for (int i = 0; i < 9; i++) {                     
    probe_data[i] = ds.read();
  }
  ds.reset_search();
  byte MSB = probe_data[1];
  byte LSB = probe_data[0];
  float Temp_Read = ((MSB << 8) | LSB);             // 2's complement
  float Temperature_Sum = Temp_Read / 16;
  return Temperature_Sum;
}

// ------------------------------------------------- pH -------------------------------------------
// This code is an adaptation of the sample test code given from dfrobot site for dfrobot pH probe -- Copyright (c) 2013 dfrobot 

float getpH() 
{
  for(int i_pH = 0; i_pH < 10; i_pH++)        // Acquire 10 sample value from the sensor for smoother result
  { 
    buf_pH[i_pH] = analogRead(Sensor_pH_Pin);
  }
  
  for(int i_pH = 0; i_pH < 9; i_pH++)             // Sort the acquired result from smallest to largest
  {
    for(int j_pH = i_pH + 1; j_pH < 10; j_pH++)
    {
      if(buf_pH[i_pH] > buf_pH[j_pH])
      {
        temp_pH = buf_pH[i_pH];
        buf_pH[i_pH] = buf_pH[j_pH];
        buf_pH[j_pH] = temp_pH;
      }
    }
  }
  Average_Value_pH = 0;
  //Average_Value_pH = analogRead(Sensor_pH_Pin);
  for(int i_pH = 2; i_pH < 8; i_pH++) 
  {
    Average_Value_pH += buf_pH[i_pH];             // Take the average value of 6 median sample
  }

  float phValue = (float)Average_Value_pH * 5.0 / 1024/6;   // Convert the average value into mV
  // one point calibration
  
  phValue = phValue-0.0114;                                   // calibration factor = 0.0114

  phValue = 3.5 * phValue;                                    // Convert the mV into pH value

  // two point calibration --> equation => y = (583/600)*x + (23/150)
  
  phValue = (phValue - (23/150))*(600/583);
 
  //float phValue = analogRead(Sensor_pH_Pin);// * 5 / 1023;
  return phValue;  
}

//------------------------------------------------- turbidity ------------------------------------------------

float getTurbidity() 
{
  Average_Value_turbidity = analogRead(Sensor_turbidity_Pin);
  float turbidityValue = Average_Value_turbidity;
  turbidityValue = turbidityValue * 4.88758553;
  return turbidityValue;
}

//=============================== Additional Procedure and Function End ================================

//********************************************* Main Loop ***********************************************
void loop(void) 
{
  Detik_Now = (millis()/1000);
  //============================================= Sensor Measurement ===========================================
  // ------------------------------------------------ DO ------------------------------------------ 
  if (Detik_Now % Sent_interval == DO_detik_count)
  {
    DO_read_ready_flag = true;
    digitalWrite(control_BJT_DO, HIGH);
    Serial3.print("C,1");
    Serial3.print('\r');
  }
  
  if ((DO_read_ready_flag == true)&&(DO_loop_count < 43))
  {
    /*Serial3.print("R");
    Serial3.print('\r');*/
    Serial.println("read_DO");
    DO_loop_count++;
    if (DO_loop_count == 39)
    {
      Serial3.print("C,0");
      Serial3.print('\r');
    }
  }
  if (sensor_DO_string_complete == true)             // Wait until DO probe return measurement result
    {
      DO_Temp_Compensation_Counter++;                 
      DO_measurement_last = sensor_DO_string.toFloat();    // DO probe will return result as a string, since it use serial to communicate with the MCU . Convert the result into a float number             
      sensor_DO_string_complete = false;
      DO_receive_data_count++;
      Serial.println(DO_measurement);
      Serial.println(DO_loop_count);      
    }
   
  if (DO_loop_count == 41)
  {
    //DO_read_ready_flag = true;
    digitalWrite(control_BJT_DO, LOW);
    DO_loop_count = 0;
    DO_receive_data_count = 0;
    DO_read_ready_flag = false;
    
  }
  
  if (DO_measurement_last != 0)
  {
    DO_measurement = DO_measurement_last;   
  }
 
  if (DO_Temp_Compensation_Counter % 5 == 0)       
  {
    sprintf(DO_Temp_Compensation_variable, "T,%s", Packet_Temperatur);   // Auto temperature compensation command for DO probe using the latest temperature measurement
    Serial3.print(DO_Temp_Compensation_variable);             // Send the command through Serial port 3 to the DO measurement chip
    Serial3.print('\r'); 
  }
  
  //----------------------------------------- Temperature & salinity ------------------------------------//
  if (Detik_Now % Sent_interval == temperature_detik_count)
  {
    temperature_read_ready_flag = true;
    salinity_read_ready_flag = true;
    digitalWrite(control_BJT_temperature, HIGH);
    //digitalWrite(control_BJT_salinity, HIGH);
  }
  if ((temperature_read_ready_flag == true)&&(Detik_Now % Sent_interval > temperature_detik_count))
  {
      digitalWrite(control_BJT_temperature, HIGH);
      digitalWrite(control_BJT_salinity, HIGH);
      temperature_2 = getTemp();    // Call the function to measure temperature
      temperature = getTemp();    // Call the function to measure temperature
      //temperature_read_ready_flag = false;
      temperature_loop_count = 0;

      if (temperature == -9)
      {
        temperature_measurement = 25;
      }
      else
      {
        temperature = temperature + 0.7; //0.7 is offset
        temperature_measurement = temperature;
      }
      Serial.println("read temperature");
      Serial.println(temperature_measurement);
    
    //temperature_read_ready_flag = false;
    //Serial.println("read temperature");
   // Serial.println(temperature_measurement);
   // Serial.println(temperature_loop_count);
  }

  if (salinity_read_ready_flag == true)
  {
    //digitalWrite(control_BJT_salinity, HIGH);
    Salinity_Measurement_old = Salinity_Measurement;
    SalinityVoltage = ((analogRead(A9) * 4.7265625)-2879.45)/8.431;            // read the voltage from the electrode, 0.01 to prevent inifinity
    if (SalinityVoltage < 8)
    {
      Salinity_Measurement = 0;
    }
    else
    {
      Salinity_Measurement = SalinityVoltage;  
    }
    Serial.println("read salinity");
    Serial.println(Salinity_Measurement);
    Serial.println(SalinityVoltage);
    Serial.println(Detik_Now);
  }
  //--------------------------------------- pH measurement -----------------------------------//
  if (Detik_Now % Sent_interval == pH_detik_count)
  {
    ph_read_ready_flag = true;
  }
  if (ph_read_ready_flag == true)
  {
    digitalWrite(control_BJT_pH, HIGH);
    //pH_measurement = getpH();   // Call the function to measure pH
    //pH_measurement = pH_measurement + (0.003*(pH_measurement - 7))*(temperature_measurement - 25); //http://www.omega.com/Green/pdf/pHbasics_REF.pdf
      pH_loop_count = 0;
      pH_measurement = getpH();   // Call the function to measure pH
      pH_measurement = pH_measurement + (0.003*(pH_measurement - 7))*(temperature_measurement - 25); //http://www.omega.com/Green/pdf/pHbasics_REF.pdf
      Serial.println("pH_read");
      Serial.println(pH_measurement);
 }
  
  //--------------------------------------- Turbidity --------------------------------------//
  float KekeruhanVoltage = getTurbidity();
  // read the input on analog pin 8


  //--------------------------------------- Salinity ----------------------------------------//
  /*if (Detik_Now % Sent_interval == salinity_detik_count)
  {
    salinity_read_ready_flag = true;
  }
  if (salinity_read_ready_flag == true)
  {
    digitalWrite(control_BJT_salinity, HIGH);
    Salinity_Measurement_old = Salinity_Measurement;
    SalinityVoltage = ((analogRead(A9) * 4.7265625)-2879.45)/8.431;            // read the voltage from the electrode, 0.01 to prevent inifinity
    /*if (SalinityVoltage < 8)
    {
      Salinity_Measurement = 0;
    }
    else
    {
      Salinity_Measurement = SalinityVoltage;  
    }
    Serial.println("read salinity");
    Serial.println(Salinity_Measurement);
    Serial.println(SalinityVoltage);
    Serial.println(Detik_Now);
    
  }*/
 
  //============================================= Sensor Measurement End ===========================================//
  
  //============================================= LCD ==================================================//
  
 
  
    //----------------------------------------- Available Display ------------------------------------------
    if (Active_Display == 1)                               
    {
    lcd.setCursor(0, 0);
    lcd.print("Temp Read:");
    lcd.setCursor(0, 1);
    lcd.print(temperature);
    lcd.setCursor(6, 1);
    lcd.print((char)223);
    lcd.setCursor(7, 1);
    lcd.print("C");
    lcd.setCursor(12, 1);
    lcd.print(Detik_Now);
    delay(100);
    }

    if (Active_Display == 2)                             
    {
    lcd.setCursor(0, 0);
    lcd.print("D.O. Read:");
    lcd.setCursor(0, 1);
    lcd.print(DO_measurement);
    lcd.setCursor(6, 1);
    lcd.print("mg/L");
    lcd.setCursor(12, 1);
    lcd.print(Detik_Now);
    delay(100);
    }

    if (Active_Display == 3)
    {
    lcd.setCursor(0, 0);
    lcd.print("pH Read:");
    lcd.setCursor(0, 1);
    lcd.print(pH_measurement);
    lcd.setCursor(12, 1);
    lcd.print(Detik_Now);
    delay(100);
    }

    if (Active_Display == 4)
    {
    lcd.setCursor(0, 0);
    lcd.print("Salinity Read:");
    lcd.setCursor(6, 1);
    lcd.print("g/L");
    lcd.setCursor(0, 1);
    lcd.print(Salinity_Measurement);
    lcd.setCursor(12, 1);
    lcd.print(Detik_Now);
    delay(100);
    }

    if (Active_Display == 0)
    {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Bacaan Kekeruhan:");
    lcd.setCursor(0, 1);
    lcd.print(KekeruhanVoltage);
    lcd.setCursor(8, 1);
    lcd.print("mV");
    lcd.setCursor(12, 1);
    lcd.print(Detik_Now);
    delay(500);
    }
  
  
  //============================================= LCD End ==================================================
  
  
  //============================================ button mode ===============================================
  
  Button_delay = 0;
  Button_State_22 = digitalRead(Button_Pin);
  while (Button_State_22 == HIGH)
  {
    last_button_pressed = Detik_Now;
    Button_delay++;
    delay(200);
    Button_State_22 = digitalRead(Button_Pin);
    Serial.print(Button_delay);
    if (Button_delay > 7)                       // Button is long-pressed
    {
      if (LCD_state == 1)                       // If LCD backlight is currently on, turn it off
      {
        lcd.clear();
        lcd.noDisplay();  
        digitalWrite(LCDbacklight_Pin, LOW);
        delay(100);
        LCD_state = 0;
        Button_delay = 0;
      }
      else                                      // If LCD backlight is currently off, turn it on
      {
        lcd.clear();
        digitalWrite(LCDbacklight_Pin, HIGH);
        lcd.display();
        delay(100);
        LCD_state = 1;
        Button_delay = 0; 
      }
    }
    else                                        // Button is short-pressed, cycle between the display
    {
      Active_Display = (Active_Display + 1) % 5;
      lcd.clear();
      delay(100);
     }
  }

  //==========================================- transmitting ==========================================
  
  if ((Detik_Now % Sent_interval == 0)&&(sent_ready_flag == 0))      // Transmitting the Packet to the HMI every 5 second, to conserve power
  {
  
    // Put every measurement result into its own Packet part -> string, set the length of the packet part, and number of digits after the decimal sign
    dtostrf(temperature, strlen(Packet_Temperatur), 2, Packet_Temperatur);        
    dtostrf(pH_measurement, strlen(Packet_pH), 2, Packet_pH);
    dtostrf(KekeruhanVoltage, strlen(Packet_Kekeruhan), 0, Packet_Kekeruhan);
    dtostrf(DO_measurement, strlen(Packet_DO), 2, Packet_DO);
    dtostrf(Salinity_Measurement, strlen(Packet_Salinitas), 2, Packet_Salinitas);
       
    //------------------- To keep the length of the packet part thet being sent to the HMI, if the result is less than expected amount, an additional "0" is added in front of the packet part --------
    if (strlen(Packet_Temperatur) < 5)
    {
       sprintf(Packet_Temperatur_2, "0%s", Packet_Temperatur);
    }
    else
    {
      sprintf(Packet_Temperatur_2, "%s", Packet_Temperatur);
    }
    if (strlen(Packet_pH) < 5)
    {
       sprintf(Packet_pH_2, "0%s", Packet_pH);
    }
    else
    {
      sprintf(Packet_pH_2, "%s", Packet_pH);
    }

      if (strlen(Packet_Kekeruhan) < 5)
    {
       sprintf(Packet_Kekeruhan_2, "0%s", Packet_Kekeruhan);
    }
    else
    {
      sprintf(Packet_Kekeruhan_2, "%s", Packet_Kekeruhan);
    }

     if (strlen(Packet_DO) < 5)
    {
       sprintf(Packet_DO_2, "0%s", Packet_DO);
    }
    else
    {
      sprintf(Packet_DO_2, "%s", Packet_DO);
    }

    if (strlen(Packet_Salinitas) < 5)
    {
       sprintf(Packet_Salinitas_2, "0%s", Packet_Salinitas);
    }
    else
    {
      sprintf(Packet_Salinitas_2, "%s", Packet_Salinitas);
    }
    

    //--------------------------- Combine the whole packet part into a Packet that being sent to the HMI module -------------------------------------
    sprintf(Packet_Total, "%s%s%s%s%s", Packet_Temperatur_2, Packet_pH_2, Packet_DO_2, Packet_Kekeruhan_2, Packet_Salinitas_2);


    //--------------------------- Print the packet to the serial monitor for debugging ---------------------------------
    for (int i=0; i < strlen(Packet_Total); i++)
    {
          Serial.print(Packet_Total[i]);
    }
    Serial.println();
    
    if (vw_send((uint8_t *)Packet_Total, strlen(Packet_Total)))       // Check if the packet is sent
    {
      if (vw_tx_active())                                             // Turn on the LED when transmitting the data
      {digitalWrite(13, 1);}
    }
    vw_wait_tx();                                                     // Wait until the whole message is gone
    digitalWrite(13, 0);                                              // Turn off the LED after completing transmission
    sent_ready_flag = 1;
  }

  //========================================= flag management ===============================================
  if (Detik_Now % Sent_interval == 1)                                             // To ensure only 1 transmission made in 1 second interval
  {
    sent_ready_flag = 0;
  }

  if(Detik_Now % Sent_interval == (temperature_detik_count + Sensor_active_interval_1))
  {
    temperature_read_ready_flag = false;
    salinity_read_ready_flag = false;
    digitalWrite(control_BJT_temperature, LOW);
    //digitalWrite(control_BJT_salinity, LOW);
  }

  if(Detik_Now % Sent_interval == (pH_detik_count + Sensor_active_interval_2))
  {
    ph_read_ready_flag = false;
    digitalWrite(control_BJT_pH, LOW);
  }

  /*if(Detik_Now % Sent_interval == (salinity_detik_count + Sensor_active_interval))
  {
    salinity_read_ready_flag = false;
    digitalWrite(control_BJT_salinity, LOW);
  }*/
}

//********************************************* Main Loop End ***********************************************


