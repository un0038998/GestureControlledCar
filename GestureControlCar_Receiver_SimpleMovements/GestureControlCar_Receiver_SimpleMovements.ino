#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//#define PRINT_DEBUG   //Uncomment this line if you want to print the values on serial monitor

#define SIGNAL_TIMEOUT 500  // This is signal timeout in milli seconds.
#define MAX_MOTOR_SPEED 200

const uint64_t pipeIn = 0xF9E8F0F0E1LL;
RF24 radio(8, 9); 
unsigned long lastRecvTime = 0;

struct PacketData
{
  byte xAxisValue;    
  byte yAxisValue;
} receiverData;

//Right motor
int enableRightMotor=5; 
int rightMotorPin1=2;
int rightMotorPin2=3;

//Left motor
int enableLeftMotor=6;
int leftMotorPin1=4;
int leftMotorPin2=7;


void setup()
{
  pinMode(enableRightMotor,OUTPUT);
  pinMode(rightMotorPin1,OUTPUT);
  pinMode(rightMotorPin2,OUTPUT);
  
  pinMode(enableLeftMotor,OUTPUT);
  pinMode(leftMotorPin1,OUTPUT);
  pinMode(leftMotorPin2,OUTPUT);

  rotateMotor(0,0); 
    
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(1,pipeIn);
  radio.startListening(); //start the radio receiver 

  #ifdef PRINT_DEBUG
    Serial.begin(115200);
  #endif
}

void loop()
{
    int rightMotorSpeed=0;
    int leftMotorSpeed=0;
    // Check if RF is connected and packet is available 
    if(radio.isChipConnected() && radio.available())
    {
      radio.read(&receiverData, sizeof(PacketData)); 

      //We will receive value as 0 to 254. Center value is 127
      
      if (receiverData.yAxisValue >= 175)       //Move car Forward
      {
        rotateMotor(MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
      }
      else if (receiverData.yAxisValue <= 75)   //Move car Backward
      {
        rotateMotor(-MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED);
      }
      else if (receiverData.xAxisValue >= 175)  //Move car Right
      {
        rotateMotor(-MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
      }
      else if (receiverData.xAxisValue <= 75)   //Move car Left
      {
        rotateMotor(MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED);
      }
      else                                      //Stop the car
      {
        rotateMotor(0, 0);
      }
      lastRecvTime = millis();  
      
      #ifdef PRINT_DEBUG  
        Serial.println(receiverData.xAxisValue);
        Serial.println(receiverData.yAxisValue);      
      #endif
    }
    else
    {
      //Signal lost. Reset the motor to stop
      unsigned long now = millis();
      if ( now - lastRecvTime > SIGNAL_TIMEOUT ) 
      {
        rotateMotor(0, 0);   
     }
   }
}


void rotateMotor(int rightMotorSpeed, int leftMotorSpeed)
{
  if (rightMotorSpeed < 0)
  {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,HIGH);    
  }
  else if (rightMotorSpeed > 0)
  {
    digitalWrite(rightMotorPin1,HIGH);
    digitalWrite(rightMotorPin2,LOW);      
  }
  else
  {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,LOW);      
  }
  
  if (leftMotorSpeed < 0)
  {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,HIGH);    
  }
  else if (leftMotorSpeed > 0)
  {
    digitalWrite(leftMotorPin1,HIGH);
    digitalWrite(leftMotorPin2,LOW);      
  }
  else
  {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,LOW);      
  }  

  analogWrite(enableRightMotor, abs(rightMotorSpeed));
  analogWrite(enableLeftMotor, abs(leftMotorSpeed));    
}
