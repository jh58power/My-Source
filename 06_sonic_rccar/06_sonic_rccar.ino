#include <Servo.h>

int RightMotor_E_pin = 5;      // 오른쪽 모터의 Enable & PWM
int RightMotor_1_pin = 8;      // 오른쪽 모터 제어선 IN1
int RightMotor_2_pin = 9;     // 오른쪽 모터 제어선 IN2
int LeftMotor_3_pin = 10;      // 왼쪽 모터 제어선 IN3
int LeftMotor_4_pin = 11;      // 왼쪽 모터 제어선 IN4
int LeftMotor_E_pin = 6;      // 왼쪽 모터의 Enable & PWM
int speed_value_motorA = 180;
int speed_value_motorB = 180;

int outputPin = 13; // Trig
int inputPin = 12;  // Echo

int Fspeedd = 0;        //-Speed
int Rspeedd = 0;        // Right speed
int Lspeedd = 0;        // Left-speed
int directionn = 0;     // Front = 8  Left =4 back = 2 = 4 Right = 6

Servo myservo;          // Set myservo
int delay_time = 250; // settling time after steering servo motors
int Servo_Lpoint = 20;
int Servo_midpoint = 65;
int Servo_Rpoint = 140;
int Servo_pin = 2;
int pos;

int Fgo = 8;           // Forward
int Rgo = 6;           // Right
int Lgo = 4;           // Left
int Bgo = 2;           // Reverse

void setup ()
{
  Serial.begin (9600);       // Define motor output pin
  pinMode(RightMotor_E_pin, OUTPUT);        // 출력모드로 설정
  pinMode(RightMotor_1_pin, OUTPUT);
  pinMode(RightMotor_2_pin, OUTPUT);
  pinMode(LeftMotor_3_pin, OUTPUT);
  pinMode(LeftMotor_4_pin, OUTPUT);
  pinMode(LeftMotor_E_pin, OUTPUT);
  pinMode (inputPin, INPUT);      // Define ultrasound input pin
  pinMode (outputPin, OUTPUT);    // Define ultrasonic output pin
  analogWrite(RightMotor_E_pin, speed_value_motorA);
  analogWrite(LeftMotor_E_pin, speed_value_motorB);
  myservo.attach(Servo_pin);      // Define servo motor output section 2 pin (PWM)
  myservo.write (Servo_midpoint);
  delay(300);
}

void motor_speed() {
  analogWrite(RightMotor_E_pin, speed_value_motorA);
  analogWrite(LeftMotor_E_pin, speed_value_motorB);
}

void advance (int a)       // Forward
{
  motor_speed();
  digitalWrite(RightMotor_1_pin, HIGH);
  digitalWrite(RightMotor_2_pin, LOW);
  digitalWrite(LeftMotor_3_pin, HIGH);
  digitalWrite(LeftMotor_4_pin, LOW);
  delay (a * 100);
}

void right (int b)          // Turn right (single wheel)
{
  motor_speed();
  digitalWrite(RightMotor_1_pin, LOW);
  digitalWrite(RightMotor_2_pin, LOW);
  digitalWrite(LeftMotor_3_pin, HIGH);
  digitalWrite(LeftMotor_4_pin, LOW);
  delay (b * 100);
}
void left (int c)           // Turn left (single wheel)
{
  motor_speed();
  digitalWrite(RightMotor_1_pin, HIGH);
  digitalWrite(RightMotor_2_pin, LOW);
  digitalWrite(LeftMotor_3_pin, LOW);
  digitalWrite(LeftMotor_4_pin, LOW);
  delay (c * 100);
}
void turnR (int d)          // Turn right (wheel)
{
  motor_speed();
  digitalWrite(RightMotor_1_pin, LOW);
  digitalWrite(RightMotor_2_pin, HIGH);
  digitalWrite(LeftMotor_3_pin, HIGH);
  digitalWrite(LeftMotor_4_pin, LOW);
  delay (d * 100);
}
void turnL (int e)          // Turn left (wheel)
{
  motor_speed();
  digitalWrite(RightMotor_1_pin, HIGH);
  digitalWrite(RightMotor_2_pin, LOW);
  digitalWrite(LeftMotor_3_pin, LOW);
  digitalWrite(LeftMotor_4_pin, HIGH);
  delay (e * 100);
}
void stopp (int f)           // Stop
{
  motor_speed();
  digitalWrite(RightMotor_1_pin, LOW);
  digitalWrite(RightMotor_2_pin, LOW);
  digitalWrite(LeftMotor_3_pin, LOW);
  digitalWrite(LeftMotor_4_pin, LOW);
  delay (f * 100);
}
void back (int g)            // Check out
{
  motor_speed();
  digitalWrite(RightMotor_1_pin, LOW);
  digitalWrite(RightMotor_2_pin, LOW);
  digitalWrite(LeftMotor_3_pin, LOW);
  digitalWrite(LeftMotor_4_pin, LOW);
  delay (g * 100);
}

void detection ()          // Measure three angles (0.90.179)
{
  int delay_time = 1000;    // Settling time  servo motor after turning
  ask_pin_F ();              // Read from front

  if (Fspeedd < 10)          // If the distance is less than 10 cm in front of
  {
    stopp (1);                 // Clear the output data
    back (2);                  // Check out 0.2 seconds
  }

  if (Fspeedd < 25)          // If the distance is less than 25 cm in front of
  {
    stopp (1);                 // Clear the output data
    ask_pin_L ();              // Read from left
    delay (delay_time);        // Wait for a stable servo motor
    ask_pin_R ();              // Read from the right
    delay (delay_time);        // Wait for a stable servo motor

    if (Lspeedd > Rspeedd)    // If the distance is greater than the right from the left
    {
      directionn = Rgo;        // Right away
    }

    if (Lspeedd <= Rspeedd)     // If the left is less than or equal to the distance from the right
    {
      directionn = Lgo;        // Turn Left
    }

    if (Lspeedd < 10 && Rspeedd < 10)   // If the distance to the left and right are less than 10 cm distance
    {
      directionn = Bgo;        // To go after
    }
  }
  else                        // Add as front not less than (greater than) 25 cm
  {
    directionn = Fgo;          // Move forward
  }

}

void ask_pin_F ()     // Measure the distance from the front
{
  myservo.write (Servo_midpoint);
  digitalWrite (outputPin, LOW);     // Let ultrasonic transmitter low voltage 2 μ s
  delayMicroseconds (2);
  digitalWrite (outputPin, HIGH);    // Let ultrasonic transmitter high voltage 10 μ s, where at least 10 μ s
  delayMicroseconds (10);
  digitalWrite (outputPin, LOW);      // Maintain low voltage ultrasonic transmitter
  float Fdistance = pulseIn (inputPin, HIGH);    // Read worse time difference
  Fdistance = Fdistance / 5.8 / 10;     // Time to turn to the distance (unit: cm)
  Serial.print ("F distance:");        // Output distance (unit: cm)
  Serial.println (Fdistance);           // Display the distance
  Fspeedd = Fdistance;                 // Read into the distance Fspeedd (former speed)
}

void ask_pin_L ()     // Measure the distance from the left
{
  myservo.write (Servo_Lpoint);
  delay (delay_time);
  digitalWrite (outputPin, LOW);     // Let ultrasonic transmitter low voltage 2 μ s
  delayMicroseconds (2);
  digitalWrite (outputPin, HIGH);    // Let ultrasonic transmitter high voltage 10 μ s, where at least 10 μ s
  delayMicroseconds (10);
  digitalWrite (outputPin, LOW);      // Maintain low voltage ultrasonic transmitter
  float Ldistance = pulseIn (inputPin, HIGH);    // Read worse time difference
  Ldistance = Ldistance / 5.8 / 10;     // Time to turn to the distance (unit: cm)
  Serial.print ("L distance:");         // Output distance (unit: cm)
  Serial.println (Ldistance);           // Display the distance
  Lspeedd = Ldistance;                // Read into the distance Lspeedd (left-speed)
  for(pos=Servo_Lpoint ; pos<=Servo_midpoint ; pos++){
    myservo.write(pos);
    delay(20);
  }
}

void ask_pin_R ()     // Measure the distance from the right
{
  myservo.write (Servo_Rpoint);
  delay (delay_time);
  digitalWrite(outputPin, LOW);     // Let ultrasonic transmitter low voltage 2 μ s
  delayMicroseconds (2);
  digitalWrite (outputPin, HIGH);
  delayMicroseconds (10);
  digitalWrite (outputPin, LOW);      // Maintain low voltage ultrasonic transmitter
  float Rdistance = pulseIn (inputPin, HIGH);    // Read worse time difference
  Rdistance = Rdistance / 5.8 / 10;     // Time to turn to the distance (unit: cm)
  Serial.print ("R distance:");         // Output distance (unit: cm)
  Serial.println (Rdistance);           // Display the distance
  Rspeedd = Rdistance;                // Will read into the distance Rspeedd (Right-speed)
  for (pos = Servo_Rpoint; pos >= Servo_midpoint; pos--) {
    myservo.write(pos);
    delay(20);
  }
}

void loop ()
{
  myservo.write (Servo_midpoint);    // Let servo motor position ready to return to the pre-prepared next time measurement
  detection ();          // Measure the angle and direction of judgment to where to move

  if (directionn == 2)    // If directionn (direction) = 2 (reverse)
  {
    back (8);                      //    Retrogression (car)
    turnL (2);                     // Move slightly to the left (to prevent stuck in dead alley)
    Serial.print ("Reverse");     // Display direction (backwards)
  }
  if (directionn == 6)             // If directionn (direction) = 6 (right turn)
  {
    back (1);
    turnR (6);                     // Right
    Serial.print ("Right");      // Display direction (turn left)
  }
  if (directionn == 4)            // If directionn (direction) = 4 (turn left)
  {
    back (1);
    turnL (6);                    // Left
    Serial.print ("Left");       // Display direction (turn right)
  }
  if (directionn == 8)            // If directionn (direction) = 8 (forward)
  {
    advance (1);                   // Normal Forward
    Serial.print ("Advance");     // Display direction (forward)
    Serial.print ("     ");
  }
}
