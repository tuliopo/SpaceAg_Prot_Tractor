#include <SoftwareSerial.h>
SoftwareSerial mySerial(10, 11);  //rx,tx
 #include <CircularBuffer.h>

#include <math.h>
long unsigned int counterPulse = 0;
#define LED_B_PIN 5
#define LED_G_PIN A2
#define ROTARY_SENSOR_PIN 2
#define TANK1_PIN   A7
#define TANK2_PIN   A6
#define BUZZER_PIN 4
#define RELAY_PIN 10

#define BUFFER_SIZE   30
 //0.6v ADC 128 ->Vacio
 //0.0167v ADC 8 ->lleno
#define TANK_2_MIN_ADC 1 
#define TANK_2_MAX_ADC 384

#define TANK_1_MIN_ADC 1 
#define TANK_1_MAX_ADC 489
#define TRACTOR_WHEEL_D   (float)1.5    //meters
#define FATOR_RPM_VELOCIDAD ((float)(3.14*TRACTOR_WHEEL_D*3.6)) //multiply by the frequency
CircularBuffer<float,BUFFER_SIZE> tank1Buffer,tank2Buffer;
#define SET_LED_B() digitalWrite(LED_B_PIN,HIGH)
#define CLR_LED_B() digitalWrite(LED_B_PIN,LOW)
#define SET_LED_G() digitalWrite(LED_G_PIN,HIGH)
#define CLR_LED_G() digitalWrite(LED_G_PIN,LOW)
#define SET_BUZZER() digitalWrite(BUZZER_PIN,HIGH)
#define CLR_BUZZER() digitalWrite(BUZZER_PIN,LOW)

#define SET_RELAY() digitalWrite(RELAY_PIN,HIGH)
#define CLR_RELAY() digitalWrite(RELAY_PIN,LOW)
#define VALVE_TURN_OFF() SET_RELAY()
#define VALVE_TURN_ON() CLR_RELAY()
#define VALVE_SHUTOFF_DECAY_SPEED 3 //size of shut off disarm usually we want it to be faster than turn off. 3=> turns on 3 times faster than turns off
float lastSpeed =0;
int shutOffCounter = 0;
int shutOffLimit = 5;  //TIme in seconds to turn valve off if no movement is detected. It can be given by a potentiometer

volatile int speed_frequency = 0; // Measures speed sensor pulses
float km_h = 0; // Calculated k/h
unsigned long currentTime;
unsigned long cloopTime;
unsigned long updateSensorsTime;
int tank1Var=0, tank2Var=0;

void speed () // Interrupt function
{
   speed_frequency++;
}

void setup() {
  // put your setup code here, to run once:
   Serial.begin(9600);
     mySerial.begin(9600);
     pinMode(LED_B_PIN,OUTPUT);
     pinMode(LED_G_PIN,OUTPUT);
     pinMode(BUZZER_PIN,OUTPUT);
     pinMode(RELAY_PIN,OUTPUT);
    

     pinMode(ROTARY_SENSOR_PIN, INPUT_PULLUP);
     attachInterrupt(digitalPinToInterrupt(ROTARY_SENSOR_PIN), speed, RISING);
     sei(); // Enable interrupts
     currentTime = millis();
     updateSensorsTime = millis();
     cloopTime = currentTime;
       CLR_LED_B();
      VALVE_TURN_ON();
}
static bool isGreenOn =false;
bool isValveOn = false;
void loop() {
  // put your main code here, to run repeatedly:
 

currentTime = millis();
   // Every second, calculate and print litres/hour
   if(currentTime >= (cloopTime + 1000))
   {
      if(isGreenOn)
      {
         CLR_LED_G();
      }
      else
      {
        SET_LED_G();
      }
      isGreenOn = !isGreenOn;
        cloopTime = currentTime; // Updates cloopTime
  
        km_h = (speed_frequency *FATOR_RPM_VELOCIDAD); // (Pulse frequency x FATOR_RPM_VELOCIDAD) 
        
        speed_frequency = 0; // Reset Counter
      if(km_h == 0) //if tractor is not moving
     {
        if(shutOffLimit > shutOffCounter)
          {
            shutOffCounter++;
          }
         else //if not moving for a while stop the valve
         {
            VALVE_TURN_OFF();
            if(isValveOn == true)
          {
            SET_BUZZER();
            delay(700);
            CLR_BUZZER();

          }
            isValveOn = false;
         }
     }
     else //if tractor is moving
     {
        if(shutOffCounter > VALVE_SHUTOFF_DECAY_SPEED)
        {
          shutOffCounter -= VALVE_SHUTOFF_DECAY_SPEED;
        }
        else
        {
          VALVE_TURN_ON();
          if(isValveOn == false)
          {
            for(int i=0;i<3;i++)
            {
              SET_BUZZER();
              delay(100);
              CLR_BUZZER();
              delay(100);

            }
          }
          isValveOn = true; 
        }
     }
   }
   
  
  lastSpeed = km_h;
    if(currentTime >= (updateSensorsTime + 200))
   {
      updateSensors();

      updateSensorsTime = millis();
    }
 
}


void updateSensors()
{
  readTank1();
  readTank2();
  tank1Buffer.push(tank1Var);
  tank2Buffer.push(tank2Var);

  if(tank2Buffer.isFull())
  {
    
      String dataStr = "";
      dataStr  +=  String(getAverage(tank1Buffer) ) + "\t" + String(getAverage(tank2Buffer)) + "\t" +String(km_h) ;
      Serial.println(dataStr);
      mySerial.println(dataStr);
  }
}


float readTank1()
{
  tank1Var = analogRead(TANK1_PIN);
  tank1Var =100- map(tank1Var,TANK_1_MIN_ADC,TANK_1_MAX_ADC,0,100);
  return tank1Var;
}


float readTank2()
{
  tank2Var = analogRead(TANK2_PIN);
  tank2Var = 100 - map(tank2Var,TANK_2_MIN_ADC,TANK_2_MAX_ADC,0,100);
  return tank2Var;
}

float getAverage(auto  buffer)
{
  float avg =0;
  for(int i=0;i<BUFFER_SIZE;i++)
  {
    avg += buffer[i]/(float)BUFFER_SIZE;
  }
  return avg;
}



