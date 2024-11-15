#ifndef __CONFIG_H__
#define __CONFIG_H__

const int motorPin1 = 2;  
const int motorPin2 = 3;  
const int redPin    = 14;    
const int greenPin  = 15;  
const int bluePin   = 16;   
const int sensPin   = 17;   

const int ESC_SERVO_RATE = 400; 
const float ZERO_THRESHOLD = 0.1;  

const int PWM_1000us = 65536 * 40 / 100;  
const int PWM_2000us = 65536 * 80 / 100;  

const float boostThreshold = 0.2;        
const float boostSpeed = 0.5;            
const unsigned long baseCycleTime = 50;  
const unsigned long coolDownTime = 100;

const float RPS_THRESHOLD = 7.0;              
const unsigned long LOOP_DELAY_MICROS = 200;  

#endif