#include <PID_v1.h>
#include <ros.h>
#include <rospy_tutorials/Floats.h>
#include <std_msgs/UInt16.h>

#define PULSES_PER_TURN (38000.0)   // 2096 x 4  Encoder Resolution:  CPR *4  for convert to PPC
                                   // from manual turn wheel ~38000 , ~38100   

#define encoderPinA1      19                       // Quadrature encoder A pin
#define encoderPinB1      18                       // Quadrature encoder B pin

#define encoderPinA2      20                       // Quadrature encoder A pin
#define encoderPinB2      21                       // Quadrature encoder B pin

#define M1              5                       // PWM outputs to L298N H-Bridge motor driver module
#define M2              6

#define M3              7
#define M4              8

ros::NodeHandle  nh;
rospy_tutorials::Floats right_joint_state, left_joint_state;


int motorPWM = 0;

boolean Direction ;//the rotation direction 

long previousMillis = 0;
long currentMillis = 0;


//-----------------------------------------------------------------------------

double  kp =1, ki =20 , kd =0;             // ki=20 modify for optimal performance
double  inputRight = 0, inputLeft = 0, outputRight = 0, outputLeft = 0 ;
double  setpointRight = 0 , setpointLeft = 0;

unsigned long lastTime,now;
volatile long encoderPosRight = 0, encoderPosLeft = 0 ;
volatile long last_pos_right=0,lastpos_right=0,last_pos_left=0,lastpos_left=0;

double pos = 0;

PID myPID_Right(&inputRight, &outputRight, &setpointRight, kp, ki, kd,DIRECT);  
PID myPID_Left(&inputLeft, &outputLeft, &setpointLeft, kp, ki, kd,DIRECT); 

ros::Publisher rightPub("right_ticks", &right_joint_state);
ros::Publisher leftPub("left_ticks", &left_joint_state);

void set_right_motor(const rospy_tutorials::Floats& cmd_msg)
{
  uint8_t a = cmd_msg.data[0];
  uint8_t b = cmd_msg.data[1];
  setpointRight = (double)((b << 8 )| a );
  
}
void set_left_motor(const rospy_tutorials::Floats& cmd_msg)
{
  uint8_t a = cmd_msg.data[0];
  uint8_t b = cmd_msg.data[1];
  setpointLeft = (double)((b << 8 )| a );
}

ros::Subscriber<rospy_tutorials::Floats> subRight("/joint_right_to_aurdino", set_right_motor);
ros::Subscriber<rospy_tutorials::Floats> subLeft("/joint_left_to_aurdino", set_left_motor);

void pwm_cb( const std_msgs::UInt16& cmd_msg){
  motorPWM = cmd_msg.data; //set servo angle, should be from 0-180  
  
}

ros::Subscriber<std_msgs::UInt16> sub("pwm_to_arduino", pwm_cb);

void setup()
{
  
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);

  pinMode(M3, OUTPUT);
  pinMode(M4, OUTPUT);
  
  right_joint_state.data = (float *)malloc(sizeof(float)*2);
  right_joint_state.data_length = 2;

  left_joint_state.data = (float *)malloc(sizeof(float)*2);
  left_joint_state.data_length = 2;
  
  
  // ROS Setup
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(rightPub);
  nh.advertise(leftPub);

  nh.subscribe(subRight);
  nh.subscribe(subLeft);

  nh.subscribe(sub);
  

  TCCR1B = TCCR1B & 0b11111000 | 1;                   // To prevent Motor Noise
  
  myPID_Right.SetMode(AUTOMATIC);
  myPID_Right.SetSampleTime(1);
  myPID_Right.SetOutputLimits(-255, 255);

  myPID_Left.SetMode(AUTOMATIC);
  myPID_Left.SetSampleTime(1);
  myPID_Left.SetOutputLimits(-255, 255);
 
  EncoderInit();//Initialize the module
  
}
 
void loop()
{
    
 
     now = millis();
     currentMillis = millis();

     if((currentMillis - lastTime) > 500 )
   {
      lastTime=now;
      
      inputRight = (360.0*1000*(encoderPosRight-last_pos_right)) /( PULSES_PER_TURN *(now - lastTime));
      inputLeft  = (360.0*1000*(encoderPosLeft -last_pos_left)) /( PULSES_PER_TURN *(now - lastTime));
      
      last_pos_right=encoderPosRight;
      last_pos_left =encoderPosLeft;
      
   }

    if( (currentMillis - lastTime) > 100 ){
   
      pos = (360.0*(encoderPosRight-lastpos_right))/PULSES_PER_TURN;
      lastpos_right=encoderPosRight;
      
      right_joint_state.data[0]=pos;
      right_joint_state.data[1]=0;

      pos = (360.0*(encoderPosLeft-lastpos_left))/PULSES_PER_TURN;
      lastpos_left=encoderPosLeft;
      
      left_joint_state.data[0]=pos;
      left_joint_state.data[1]=0;

      leftPub.publish(&left_joint_state);
      rightPub.publish(&right_joint_state);

      lastTime=now;
      
       nh.spinOnce();
      
    }

    
    //  encoderPos--;
    // if( encoderPos > (2096 * 4) ) encoderPos=0;
     
  
     myPID_Right.Compute();                                    // calculate new output
     myPID_Left.Compute(); 
     pwmRightOut(outputRight);                                     // drive L298N H-Bridge module
     pwmLeftOut(outputLeft); 
     delay(10);

}


void EncoderInit()
{
  
  pinMode(encoderPinA1,INPUT);  
  pinMode(encoderPinB1,INPUT);
  
  pinMode(encoderPinA2,INPUT);  
  pinMode(encoderPinB2,INPUT);
  
  attachInterrupt( digitalPinToInterrupt(encoderPinA1), wheelSpeedRightA1, CHANGE );
  attachInterrupt( digitalPinToInterrupt(encoderPinB1), wheelSpeedRightB1, CHANGE );

  attachInterrupt( digitalPinToInterrupt(encoderPinA2), wheelSpeedRightA2, CHANGE );
  attachInterrupt( digitalPinToInterrupt(encoderPinB2), wheelSpeedRightB2, CHANGE );

 
}
void wheelSpeedRightA1()
{

  if (digitalRead(encoderPinA1) != digitalRead(encoderPinB1))
  {
    encoderPosRight++;
  }
  else
  {
    encoderPosRight--;
  }

  
}
void wheelSpeedRightB1()
{

  if (digitalRead(encoderPinA1) == digitalRead(encoderPinB1))
  {
    encoderPosRight++;
  }
  else
  {
    encoderPosRight--;
  }

  
}

void wheelSpeedRightA2()
{

  if (digitalRead(encoderPinA2) != digitalRead(encoderPinB2))
  {
    encoderPosLeft--;
  }
  else
  {
    encoderPosLeft++;
  }

  
}
void wheelSpeedRightB2()
{

  if (digitalRead(encoderPinA2) == digitalRead(encoderPinB2))
  {
    encoderPosLeft--;
  }
  else
  {
    encoderPosLeft++;
  }

  
}


void pwmRightOut(float out) {                                
  if (out > 0) {

    analogWrite(M1, 0);                             // drive motor CW
    analogWrite(M2, out);
  }
  else {
    analogWrite(M1, abs(out));
    analogWrite(M2, 0);                        // drive motor CCW
  }
}
void pwmLeftOut(float out) {                                
  if (out > 0) {
    analogWrite(M3, out);                             // drive motor CW
    analogWrite(M4, 0);
  }
  else {
    analogWrite(M3, 0);
    analogWrite(M4, abs(out));                        // drive motor CCW
  }
}
