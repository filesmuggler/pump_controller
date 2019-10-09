/*
*  LCD 16x2 I2C & Arduino Uno
*  VCC - > 5 V
*  GND - GND
*  SCL -> A5
*  SDA -> A4
*/

#include <LCD.h>
#include <LiquidCrystal.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

#define BAUDRATE 9600
#define UP_BUTTON 12
#define DOWN_BUTTON 8
#define SAVE_BUTTON 7
#define RECALL_BUTTON 4
#define PWM_PIN_A 3
#define PWM_PIN_B 11

int perc_array[2] = {0,0};
int save_array_index = 0;
int recall_array_index = 0;
int current_perc_val = 0;
byte ab_value = 0;

long previousMillis = 0;
long interval = 100;
LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3,POSITIVE);

void setup() {
  // put your setup code here, to run once:
  
  pinMode(PWM_PIN_A,OUTPUT);
  digitalWrite(PWM_PIN_A,LOW);
  pinMode(PWM_PIN_B,OUTPUT);
  digitalWrite(PWM_PIN_B,LOW);

  // setting the PWM register on pin 3
  // frequency: 976.5625Hz
  // https://www.arduino.cc/en/Tutorial/SecretsOfArduinoPWM
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS22);
  OCR2A = 0;
  OCR2B = 0;

  pinMode(UP_BUTTON,INPUT);
  pinMode(DOWN_BUTTON,INPUT);
  pinMode(SAVE_BUTTON,INPUT);
  pinMode(RECALL_BUTTON,INPUT);
  Serial.begin(BAUDRATE);
  
  lcd.begin(16,2);
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Pump tester"); 
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;    
    if(digitalRead(UP_BUTTON)){
      if(current_perc_val >= 100){
        current_perc_val = 100;
      }
      else{
        current_perc_val = current_perc_val + 1;
      }      
      //Serial.println(current_perc_val);
      pwmLCD(current_perc_val);
      pwmSet(current_perc_val);
    }
    else if(digitalRead(DOWN_BUTTON)){
      
      if(current_perc_val <= 0){
        current_perc_val = 0;
      }
      else{
        current_perc_val = current_perc_val - 1;
      }     
      //Serial.println(current_perc_val);
      pwmLCD(current_perc_val);
      pwmSet(current_perc_val);
    }
    else if(digitalRead(SAVE_BUTTON)){
      pwmSave(current_perc_val);
      pwmLCDSave(current_perc_val);
      delay(1000);
      pwmLCD(current_perc_val);
    }
    else if(digitalRead(RECALL_BUTTON)){
      current_perc_val = pwmRecall();
      pwmLCD(current_perc_val);
    }
    
  } 
  

  
}


void pwmSet(int pwm_value){
   // set PWM
   if(pwm_value >= 1){
    ab_value = (pwm_value * 256 / 100) - 1;
   }
   else{
    ab_value = (pwm_value * 256 / 100);
   }
   
   OCR2A = ab_value;
   OCR2B = ab_value;
}

void pwmSave(int pwm_value){
  if(save_array_index == 0){
    perc_array[save_array_index] = pwm_value;
    save_array_index = 1;
  }
  else{
    perc_array[save_array_index] = pwm_value;
    save_array_index = 0;
  }
}

int pwmRecall(){
  int r_val=0;
  if(recall_array_index == 0){
    r_val = perc_array[recall_array_index];
    recall_array_index = 1;
  }
  else{
    r_val = perc_array[recall_array_index];
    recall_array_index = 0;
  }
  return r_val;
}

void pwmLCD(int pwm_value){
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("PWM %");
    lcd.setCursor(0,1);
    lcd.print(pwm_value);
}

void pwmLCDSave(int pwm_value){
  lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("PWM saved");
    lcd.setCursor(0,1);
    lcd.print(pwm_value);
}
