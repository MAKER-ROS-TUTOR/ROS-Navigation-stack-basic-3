#define encoderPinA1      19                       // Quadrature encoder A pin
#define encoderPinB1      18                       // Quadrature encoder B pin

#define encoderPinA2      20                       // Quadrature encoder A pin
#define encoderPinB2      21     

#define M1              5                       // PWM outputs to L298N H-Bridge motor driver module
#define M2              6

#define M3              7
#define M4              8

#define ENC_COUNT_REV   38000

#define PWM             0

// One-second interval for measurements
int interval = 1000;
  
// Counters for milliseconds during interval
long previousMillis = 0;
long currentMillis = 0;
 
// Variable for RPM measuerment
float rpm_right = 0;
 
// Variable for angular velocity measurement
float ang_velocity_right = 0;
float ang_velocity_right_deg = 0;
 
const float rpm_to_radians = 0.10471975512;
const float rad_to_deg = 57.29578;

volatile long encoderPosRight = 0, encoderPosLeft = 0 ;
void setup() {
  Serial.begin(9600);

  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);

  pinMode(M3, OUTPUT);
  pinMode(M4, OUTPUT);

  
  // put your setup code here, to run once:
  EncoderInit();

  analogWrite(M1, 0);                             // drive motor CW
  analogWrite(M2, PWM);

  analogWrite(M3, 0);                             // drive motor CW
  analogWrite(M4, PWM);  
}

void loop() {
  // put your main code here, to run repeatedly:
  // Record the time
  currentMillis = millis();
 
  // If one second has passed, print the number of pulses
  if (currentMillis - previousMillis > interval) {
 
    previousMillis = currentMillis;
 
    // Calculate revolutions per minute
    rpm_right = (float)(encoderPosLeft * 60 / ENC_COUNT_REV);
    ang_velocity_right = rpm_right * rpm_to_radians;   
    ang_velocity_right_deg = ang_velocity_right * rad_to_deg;
     
    Serial.print(" Pulses: ");
    Serial.println(encoderPosRight);
    Serial.print(" Speed: ");
    Serial.print(rpm_right);
    Serial.println(" RPM");
    Serial.print(" Angular Velocity: ");
    Serial.print(rpm_right);
    Serial.print(" rad per second");
    Serial.print("\t");
    Serial.print(ang_velocity_right_deg);
    Serial.println(" deg per second");
    Serial.println();
 
    encoderPosLeft = 0;
   
  }
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
