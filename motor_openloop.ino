//Import Basic Libraries
#include <ros.h>
#include <std_msgs/Float32.h>

//Declare ROS node
ros::NodeHandle nh;

//Declare variables and GPIO
const int PinM1 = 4;
const int PinM2 = 2;

const int M1_IN1 = 18;
const int M1_IN2 = 15;
const int M2_IN1 = 14;
const int M2_IN2 = 13;

const int EncA1 = 34;
const int EncB1 = 36;
const int EncA2 = 39;
const int EncB2 = 35;

const int Channel = 0;
const int freq = 980;
const int resolution = 8;

float pwm = 0.0;
bool reverse = true;

const int PPR = 11;
const float gears = 34;
volatile int count = 0;
volatile int lastCount = 0;
unsigned long lastTime = 0;
float RPS = 0.0;
float angVelocity = 0.0;
int signVelocity = 1;

//Callback function for the subscription
void callback(const std_msgs::Float32& msg){
  pwm = msg.data * 100;
  if (pwm < 0){
    reverse = true;
    signVelocity = -1;
    pwm *= (-1);
  }
  else{
    reverse = false;
    signVelocity = 1;
  }
}

//Publisher & Subscribers
std_msgs::Float32 speedMotor;
ros::Subscriber<std_msgs::Float32> sub("motor_input", &callback);
ros::Publisher pub("motor_output", &speedMotor);

void setup()
{
  Serial.begin(9600);
  pinMode(M1_IN1, OUTPUT); 
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT); 
  pinMode(M2_IN2, OUTPUT);

  pinMode(EncA1, INPUT);
  pinMode(EncB1, INPUT);
  pinMode(EncA2, INPUT);
  pinMode(EncB2, INPUT);

  //Initialize the pwm
  ledcSetup(Channel, freq, resolution);
  ledcAttachPin(PinM1, Channel);
  ledcAttachPin(PinM2, Channel);

  attachInterrupt(digitalPinToInterrupt(EncA1), pulse, RISING);

  //Manage ROS node
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);
}

void loop()
{
  //Call functions
  pwm_duty_cycle();
  directions();
  speedEncoders();

  //Publish to topics
  speedMotor.data = angVelocity * signVelocity;
  Serial.println(speedMotor.data);
  pub.publish(&speedMotor);
  nh.spinOnce();
  delay(10);
}

void pwm_duty_cycle()
{
  //Map the received pwm
  int dutyCycle = map(pwm, 0.0, 100.0, 0, 255);
  ledcWrite(Channel, dutyCycle);
}

void directions()
{
  //Control the direction for the encoders
  if (reverse){
    //ccw
    digitalWrite(M1_IN1, LOW);
    digitalWrite(M1_IN2, HIGH);
    //cw
    digitalWrite(M2_IN1, LOW);
    digitalWrite(M2_IN2, HIGH);
  }
  else{
    //cw
    digitalWrite(M1_IN1, HIGH);
    digitalWrite(M1_IN2, LOW);
    //ccw
    digitalWrite(M2_IN1, HIGH);
    digitalWrite(M2_IN2, LOW);
  }
}

void speedEncoders(){
  unsigned long currTime = millis();
  unsigned long elapsedTime = currTime - lastTime;
  
  if (elapsedTime >= 1000){
    noInterrupts();
    count = lastCount;
    lastCount = 0;
    interrupts();
    
    RPS = ((float)count / PPR) / gears;
    angVelocity = 2*3.1416*RPS;

    //Serial.print("Motor speed (RPS): ");
    //Serial.println(RPS);

    //Serial.print("Motor speed (rad/s): ");
    //Serial.println(angVelocity);
    
    lastTime = currTime;
  }
}

void pulse(){
  lastCount++;
}
