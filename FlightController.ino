#include <Wire.h>                                
test test
float p_gain_roll = 1.3;               
float i_gain_roll = 0.05;              
float d_gain_roll = 15;                
int max_roll_output = 400;                    

float p_gain_pitch = p_gain_roll;  
float i_gain_pitch = i_gain_roll;  
float d_gain_pitch = d_gain_roll;  
int max_pitch_output = max_roll_output;          

float p_gain_yaw = 4.0;                
float i_gain_yaw = 0.02;               
float d_gain_yaw = 0.0;                
int max_yaw_output = 400;                     


byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, loop_counter;
int esc_1, esc_2, esc_3, esc_4;
int throttle, battery_voltage;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
int cal_int, start;
unsigned long loop_timer;
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
byte highByte, lowByte;

float error;
float prev_i_mem_roll, roll_setpoint, gyro_roll_input, roll_output, prev_roll_d_error;
float prev_i_mem_pitch, pitch_setpoint, gyro_pitch_input, pitch_output, prev_pitch_d_error;
float prev_i_mem_yaw, yaw_setpoint, gyro_yaw_input, yaw_output, prev_yaw_d_error;


void setup(){

  Wire.begin();                                                
  
  DDRD |= B11110000;                                           
  DDRB |= B00110000;                                           
  
  
  digitalWrite(12,HIGH);                                       
  delay(3000);                                                 
  
  Wire.beginTransmission(105);                                 
  Wire.write(0x20);                                            
  Wire.write(0x0F);                                            
  Wire.endTransmission();                                      

  Wire.beginTransmission(105);                                 
  Wire.write(0x23);                                            
  Wire.write(0x90);                                            
  Wire.endTransmission();                                      

  delay(250);                                                  //Give the gyro time to start.

  
  for (cal_int = 0; cal_int < 2000 ; cal_int ++)
  {              
    if(cal_int % 15 == 0)
    {
      digitalWrite(12, !digitalRead(12));   
    }

    gyro_signalen();                                           
    gyro_roll_cal += gyro_roll;                                
    gyro_pitch_cal += gyro_pitch;                              
    gyro_yaw_cal += gyro_yaw;                                  
    PORTD |= B11110000;                                        
    delayMicroseconds(1000);                                   
    PORTD &= B00001111;                                        
    delay(3);                                                  
  }
  
  gyro_roll_cal /= 2000;                                       
  gyro_pitch_cal /= 2000;                                      
  gyro_yaw_cal /= 2000;                                        
  
  PCICR  |= (1<<PCIE0);
  PCMSK0 |= (1<<PCINT0) | (1 << PCINT1) | (1 << PCINT2) | (1 << PCINT3); 

  
  while(receiver_input_channel_3 > 1216 || receiver_input_channel_3 == 0)
  {
    start ++;                                                  
    PORTD |= B11110000;                                        
    delayMicroseconds(1000);                                   
    PORTD &= B00001111;                                        
    delay(3);                                                  
    if(start == 125)
    {                                          
      digitalWrite(12, !digitalRead(12));                      
      start = 0;                                               
    }
  }

  start = 0;                                                   
  
  //Load the battery voltage to the battery_voltage variable.
  //65 is the voltage compensation for the diode.
  //12.6V equals ~5V @ Analog 0.
  //12.6V equals 1023 analogRead(0).
  //1260 / 1023 = 1.2317.
  //The variable battery_voltage holds 1050 if the battery voltage is 10.5V.
  // battery_voltage = (analogRead(0) + 65) * 1.2317;
  
  //When everything is done, turn off the led.
  digitalWrite(12,LOW);                                        //Turn off the warning led.
}

void loop()
{
  
  gyro_signalen();
  gyro_roll_input = (gyro_roll_input * 0.8) + ((gyro_roll * 0.0175) * 0.2);            
  gyro_pitch_input = (gyro_pitch_input * 0.8) + ((gyro_pitch * 0.0175) * 0.2);         
  gyro_yaw_input = (gyro_yaw_input * 0.8) + ((gyro_yaw * 0.0175) * 0.2);               

  //For starting the motors: throttle low and yaw left (step 1).
  if(receiver_input_channel_3 < 1216 && receiver_input_channel_4 < 1216)
  {
    start = 1;
  }
  //When yaw stick is back in the center position start the motors (step 2).
  if(start == 1 && receiver_input_channel_4 > 1484)
  {
    start = 2;
    //Reset the pid controllers for a bumpless start.
    prev_i_mem_roll = 0;
    prev_roll_d_error = 0;
    prev_i_mem_roll = 0;
    prev_pitch_d_error = 0;
    prev_i_mem_yaw = 0;
    prev_yaw_d_error = 0;
  }

  //Stopping the motors: throttle low and yaw right.
  if(start == 2 && receiver_input_channel_3 < 1216 && receiver_input_channel_4 > 1784)
  {
    start = 0;
  }
  
  //In the case of dividing by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  roll_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if(receiver_input_channel_1 > 1508)
  {
    roll_setpoint = (receiver_input_channel_1 - 1508)/3.0;
  }

  else if(receiver_input_channel_1 < 1492)
  {
    roll_setpoint = (receiver_input_channel_1 - 1492)/3.0;
  }
  
  pitch_setpoint = 0;

  if(receiver_input_channel_2 > 1508)
  {
    pitch_setpoint = (receiver_input_channel_2 - 1508)/3.0;
  }

  else if(receiver_input_channel_2 < 1492)
  {
    pitch_setpoint = (receiver_input_channel_2 - 1492)/3.0;
  }
  
  yaw_setpoint = 0;
  
  if(receiver_input_channel_3 > 1050)
  {
   //Do not yaw when turning off the motors.
    if(receiver_input_channel_4 > 1508)
    {
      yaw_setpoint = (receiver_input_channel_4 - 1508)/3.0;
    }

    else if(receiver_input_channel_4 < 1492)
    {
      yaw_setpoint = (receiver_input_channel_4 - 1492)/3.0;
    }

  }
  
  calculate_pid();
  
  //The battery voltage is needed for compensation.
  //A complementary filter is used to reduce noise.
  //0.09853 = 0.08 * 1.2317.
  // battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 65) * 0.09853;
  
  //Turn on the led if battery voltage is to low.
  // if(battery_voltage < 1050 && battery_voltage > 600)digitalWrite(12, HIGH);
  
  throttle = receiver_input_channel_3;                                      //We need the throttle signal as a base signal.
  
  if (start == 2)
  {                                                          
    if (throttle > 1800) throttle = 1800;                                   
    esc_1 = throttle + pitch_output + roll_output - yaw_output; 			   //esc 1 (front-right - CCW)
    esc_2 = throttle - pitch_output + roll_output + yaw_output; 			   //esc 2 (rear-right - CW)
    esc_3 = throttle - pitch_output - roll_output - yaw_output; 			   //esc 3 (rear-left - CCW)
    esc_4 = throttle + pitch_output - roll_output + yaw_output; 			   //esc 4 (front-left - CW)

    // if (battery_voltage < 1240 && battery_voltage > 800)
    // {                   //Is the battery connected?
    //   esc_1 += esc_1 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-1 pulse for voltage drop.
    //   esc_2 += esc_2 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-2 pulse for voltage drop.
    //   esc_3 += esc_3 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-3 pulse for voltage drop.
    //   esc_4 += esc_4 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-4 pulse for voltage drop.
    // } 
    
    if(esc_1 < 1200) esc_1 = 1200;                                         
    if(esc_2 < 1200) esc_2 = 1200;                                         
    if(esc_3 < 1200) esc_3 = 1200;                                         
    if(esc_4 < 1200) esc_4 = 1200;                                         
    
    if(esc_1 > 2000) esc_1 = 2000;                                           
    if(esc_2 > 2000) esc_2 = 2000;                                           
    if(esc_3 > 2000) esc_3 = 2000;                                           
    if(esc_4 > 2000) esc_4 = 2000;                                           
  }
  
  else
  {
    esc_1 = 1000;                                                           
    esc_2 = 1000;                                                           
    esc_3 = 1000;                                                           
    esc_4 = 1000;                                                           
  }
  
  while(micros() - loop_timer < 4000);                                      
  loop_timer = micros();                                                    

  PORTD |= B11110000;                                                       
  timer_channel_1 = esc_1 + loop_timer;                                     
  timer_channel_2 = esc_2 + loop_timer;                                     
  timer_channel_3 = esc_3 + loop_timer;                                     
  timer_channel_4 = esc_4 + loop_timer;                                     
  
  while(PORTD >= 16){                                                       
    esc_loop_timer = micros();                                              
    if(timer_channel_1 <= esc_loop_timer)PORTD &= B11101111;                
    if(timer_channel_2 <= esc_loop_timer)PORTD &= B11011111;                
    if(timer_channel_3 <= esc_loop_timer)PORTD &= B10111111;                
    if(timer_channel_4 <= esc_loop_timer)PORTD &= B01111111;                
  }
}

ISR(PCINT0_vect){
  current_time = micros();
  //Channel Roll=========================================
  if(PINB & B00000001){                                        
    if(last_channel_1 == 0){                                   
      last_channel_1 = 1;                                      
      timer_1 = current_time;                                  
    }
  }
  else if(last_channel_1 == 1){                                
    last_channel_1 = 0;                                        
    receiver_input_channel_1 = current_time - timer_1;         
  }
  //Channel Pitch=========================================
  if(PINB & B00000010){                                       
    if(last_channel_2 == 0){                                  
      last_channel_2 = 1;                                     
      timer_2 = current_time;                                 
    }
  }
  else if(last_channel_2 == 1){                               
    last_channel_2 = 0;                                       
    receiver_input_channel_2 = current_time - timer_2;        
  }
  //Channel Throttle=========================================
  if(PINB & B00000100 ){                                      
    if(last_channel_3 == 0){                                  
      last_channel_3 = 1;                                     
      timer_3 = current_time;                                 
    }
  }
  else if(last_channel_3 == 1){                               
    last_channel_3 = 0;                                       
    receiver_input_channel_3 = current_time - timer_3;        
  }
  //Channel Yaw=========================================
  if(PINB & B00001000 ){                                      
    if(last_channel_4 == 0){                                  
      last_channel_4 = 1;                                     
      timer_4 = current_time;                                   
    }
  }
  else if(last_channel_4 == 1){                                
    last_channel_4 = 0;                                        
    receiver_input_channel_4 = current_time - timer_4;         
  }
}


void gyro_signalen(){
  Wire.beginTransmission(0x69);                      
  Wire.write(168);                                   
  Wire.endTransmission();                            
  Wire.requestFrom(105, 6);                          
  while(Wire.available() < 6);                       
  lowByte = Wire.read();                             
  highByte = Wire.read();                           
  gyro_pitch = ((highByte<<8)|lowByte);             
  if(cal_int == 2000)gyro_pitch -= gyro_pitch_cal;
  gyro_pitch *= -1;     
  lowByte = Wire.read();                             
  highByte = Wire.read();                            
  gyro_roll = ((highByte<<8)|lowByte);              
  if(cal_int == 2000)gyro_pitch -= gyro_roll_cal;   
  lowByte = Wire.read();                             
  highByte = Wire.read();                            
  gyro_yaw = ((highByte<<8)|lowByte);                
  gyro_yaw *= -1;                                    
  if(cal_int == 2000)gyro_yaw -= gyro_yaw_cal;       
}

void calculate_pid()
{
  //Roll calculations
  error = gyro_roll_input - roll_setpoint;
  prev_i_mem_roll += i_gain_roll * error;
  if(prev_i_mem_roll > max_roll_output)prev_i_mem_roll = max_roll_output;
  else if(prev_i_mem_roll < max_roll_output * -1)prev_i_mem_roll = max_roll_output * -1;
  
  roll_output = p_gain_roll * error + prev_i_mem_roll + d_gain_roll * (error - prev_roll_d_error);
  if(roll_output > max_roll_output)roll_output = max_roll_output;
  else if(roll_output < max_roll_output * -1)roll_output = max_roll_output * -1;
  
  prev_roll_d_error = error;
  
  //Pitch calculations
  error = gyro_pitch_input - pitch_setpoint;
  prev_i_mem_roll += i_gain_pitch * error;
  if(prev_i_mem_roll > max_pitch_output)prev_i_mem_roll = max_pitch_output;
  else if(prev_i_mem_roll < max_pitch_output * -1)prev_i_mem_roll = max_pitch_output * -1;
  
  pitch_output = p_gain_pitch * error + prev_i_mem_roll + d_gain_pitch * (error - prev_pitch_d_error);
  if(pitch_output > max_pitch_output)pitch_output = max_pitch_output;
  else if(pitch_output < max_pitch_output * -1)pitch_output = max_pitch_output * -1;
    
  prev_pitch_d_error = error;
    
  //Yaw calculations
  error = gyro_yaw_input - yaw_setpoint;
  prev_i_mem_yaw += i_gain_yaw * error;
  if(prev_i_mem_yaw > max_yaw_output)prev_i_mem_yaw = max_yaw_output;
  else if(prev_i_mem_yaw < max_yaw_output * -1)prev_i_mem_yaw = max_yaw_output * -1;
  
  yaw_output = p_gain_yaw * error + prev_i_mem_yaw + d_gain_yaw * (error - prev_yaw_d_error);
  if(yaw_output > max_yaw_output)yaw_output = max_yaw_output;
  else if(yaw_output < max_yaw_output * -1)yaw_output = max_yaw_output * -1;
    
  prev_yaw_d_error = error;
}


