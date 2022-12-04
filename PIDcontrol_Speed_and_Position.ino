#define BLYNK_PRINT Serial
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <Ticker.h>
//char auth[] = "RnAFqNV5NbWi8-OdGjxUl2Ow0s0RTEps"; // Nhập AuthToken
char auth[] = "70O4dWPgxSZLQmmA-RnrQ8jZMHDWFzVY";
char ssid[] = "Thuyendeptrai"; // Nhập tên WiFi
char pass[] = "sasageyo";   // Nhập password WiFi
//**********************************************************
//*************************WIRING***************************
//**********************************************************
// PWM2      <->    D1
// PWM1      <->    D2
// ENCB(C2)  <->    D6
// ENCA(C1)  <->    D5
// ENBALE    <->    D0 
//**********************************************************
//******************** DEFINE******************************
//**********************************************************
#define Ts 0.1
#define res 495
#define PWM1 D2
#define PWM2 D1
#define ENCA D5
#define ENCB D6
#define ENABLE_PIN D0


//**********************************************************
//*******************DECLARATION****************************
//**********************************************************
float SPEED_KP = 0.025;
float SPEED_KI = 0.5;
float SPEED_KD = 0;

float POSITION_KP = 0.35;
float POSITION_KI = 0;
float POSITION_KD = 0.0001;

int encoderPos = 0;
int _speed, _position;
int enable;
int cur_mode, pre_mode; 
float error, pre_error=0, error_sum, d_error;
float control_signal;
int flag_speed =1, flag_position=0;
int setpoint;
int reset_position_signal ,reset_speed_signal;
int cnt = 0;
//**********************************************************
//*******************CODING*********************************
//**********************************************************
Ticker myTic;   // Ticker to calc speed


BLYNK_WRITE(V0) // get speed_d from blynk
{
  if (digitalRead(ENABLE_PIN) == HIGH)
  {
    setpoint = param.asInt();
  }
}

BLYNK_WRITE(V1) // get speed_d from blynk
{
  cur_mode = param.asInt();
  // mode = 1: Speed control
  // mode = 2: Position control
  if ((cur_mode == 1)&&(pre_mode == 2))
  {
        reset_position();
      Serial.println("Change to Speed control ");
  }
  if ((cur_mode == 2)&&(pre_mode == 1))
  {
    reset_speed();
    
      Serial.println("Change to Position control ");
  }
  pre_mode = cur_mode; 
}

void handle_interrupt()
  {
    cnt = cnt +1;
    if (cnt ==5)
    cnt =0;

    
    //Serial.print(" Encoder Pulse: ");
   // Serial.println(encoderPos)          ;
    //Serial.print("RESET SIGNAL SPEED - POSITION: ");
   // Serial.print(reset_speed_signal) ;
   // Serial.print(reset_position_signal) ; 
   // Serial.print(" FLAG SPEED - POSITION: ");
   // Serial.print(flag_speed);
  //  Serial.print(flag_position);
  
      if((flag_speed == 1)&&(flag_position==0))
      {
      calc_speed();
      }
      else if((flag_position == 1)&&(flag_speed == 0))
      {
      calc_position();
      }
  }
  
void calc_speed()
{
if (digitalRead(ENABLE_PIN) == HIGH)
{
//***** calc PID
float my_duty_cycle = myPID(SPEED_KP,SPEED_KI,SPEED_KD,_speed,setpoint);
control_signal = my_duty_cycle*12/100;
if (my_duty_cycle >= 0)
{
  analogWrite(PWM1,(int)(my_duty_cycle/100*1023));
  analogWrite(PWM2,0);
}
else 
{
  analogWrite(PWM2,-my_duty_cycle/100*1023);
  analogWrite(PWM1,0);
}
//******
}
else{
  error_sum = 0; 
  setpoint = 0;
  analogWrite(PWM1,0);
  analogWrite(PWM2,0);
  if (cnt == 4)
  {
  Blynk.virtualWrite(V0,0);
  Blynk.virtualWrite(V4,0);
  }
}


Serial.print(" Speed_d: ");
Serial.print(setpoint);
Serial.print(" (rpm) ");
Serial.print(" Speed: ");
_speed = (float)(encoderPos/(float)Ts/(float)res*60); 
encoderPos = 0;// reset counter
Serial.print(_speed);
Serial.println(" (rpm) ");
if (cnt == 4)
{
Blynk.virtualWrite(V5,_speed);
Blynk.virtualWrite(V4,setpoint);
}


if ((reset_speed_signal == 1)&&(abs(_speed) <= 1))
  {
    flag_speed   =   0 ;
    flag_position =  1 ;
    reset_speed_signal = 0;
  }

}

void calc_position()
{
  if (digitalRead(ENABLE_PIN) == HIGH)
{
//***** calc PID
float my_duty_cycle = myPID(POSITION_KP,POSITION_KI,POSITION_KD,_position,setpoint);
control_signal = my_duty_cycle*12/100;
if (my_duty_cycle >= 0)
{
  analogWrite(PWM1,(int)(my_duty_cycle/100*1023));
  analogWrite(PWM2,0);
}
else 
{
  analogWrite(PWM2,-my_duty_cycle/100*1023);
  analogWrite(PWM1,0);
}
//******
}
else{
  error_sum =0; 
  setpoint = 0;
  analogWrite(PWM1,0);
  analogWrite(PWM2,0);
  if (cnt == 4)
  {
  Blynk.virtualWrite(V0,0);
  Blynk.virtualWrite(V4,0);
  }
}


Serial.print(" Set point: ");
Serial.print(setpoint);
Serial.print(" (degree) ");
Serial.print(" Position: ");
_position = (float)(encoderPos)*360/(float)res; 
Serial.print(_position);
Serial.println(" (degree) ");
if (cnt == 4)
{
Blynk.virtualWrite(V5,_position);
Blynk.virtualWrite(V4,setpoint);
}

if ((reset_position_signal == 1)&&(abs(_position) <= 5))
  {
    flag_speed   =  1 ;
    flag_position = 0 ;
    reset_position_signal=0;
  }

}
void reset_speed()
{
  setpoint  = 0;
  //Blynk.virtualWrite(V0,0);
 // Blynk.virtualWrite(V4,0);  
  reset_speed_signal = 1;
}

void reset_position()
{
  setpoint = 0;  
  if (cnt == 4)
  {
  Blynk.virtualWrite(V0,0);
  Blynk.virtualWrite(V4,0);
  }
  reset_position_signal = 1;
}

float myPID(float KP,float KI,float KD,float current,int setpoint)
{
  error = setpoint - current;
  error_sum += error;
  d_error = (error - pre_error);
  float duty_cycle;
  duty_cycle = KP*error + KI*Ts*error_sum + KD*d_error/Ts ; 
  pre_error = error ; 
  if (duty_cycle > 100)
    duty_cycle =  99;
  else if(duty_cycle<-100)
    duty_cycle = -99;
  return(duty_cycle);
}


void setup()
{
  Serial.begin(9600);
  Blynk.begin(auth, ssid, pass);
  pinMode(ENCA, INPUT_PULLUP);                             // quadrature encoder input A
  pinMode(ENCB, INPUT_PULLUP);                             // quadrature encoder input B
  pinMode(ENABLE_PIN, OUTPUT);
  attachInterrupt(14, ISR_encoder, FALLING);               // update encoder position
  myTic.attach(Ts, handle_interrupt);
}
void loop()
{
  Blynk.run();  
}

ICACHE_RAM_ATTR void ISR_encoder()  {                         
  if (digitalRead(ENCB)==HIGH)   encoderPos++;
  else                         encoderPos--;      
}
