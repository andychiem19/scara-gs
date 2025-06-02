#include <Stepper.h>
#include <math.h>
const int stepsPerRevolution = 2048; // for stepper motor 28BYJ-48 

// Initialize stepper motors
Stepper motor1(stepsPerRevolution, 22, 24, 23, 25);
Stepper motor2(stepsPerRevolution, 26, 28, 27, 29);
Stepper motor3(stepsPerRevolution, 30, 32, 31, 33);

//initializing electronic locking variables
long motor1_position = 0;
long motor2_position = 0;
long motor3_position = 0;

//function prototypes 
long degToSteps(float degrees);
long zToSteps(float z);

//constants
float L1 = 13.3; // 13.3 cm from base pivot axis to joint pivot axis
float L2 = 13.3; // 13.3 cm from joint pivot axis to end effector

const long verticalMinSteps = zToSteps(-18); // sets how far below neutral the end effector can move; -5 cm
const long verticalMaxSteps = zToSteps(5);

//setup function, establishes distances between pivot points
void setup() {
  Serial.begin(9600);
  motor1.setSpeed(10);
  motor2.setSpeed(10);
  motor3.setSpeed(10);

  motor1_position = degToSteps(0);  // 0°
  motor2_position = degToSteps(0);  // 0°
  motor3_position = zToSteps(0);    // 0 cm height
}

//main loop, runs the two solutions repeatedly. may need to create a more robust way to select one or the other depending on the current trial
void loop() {
  moveTo(-7,7,0,L1,L2);
  delay(1000);
  moveTo(5,7,0,L1,L2);
  delay(2000);
}

//IK function, implements trigonometry of inverse kinematics
bool inverseKinematics(float x, float y, float L1, float L2, float &theta1, float &theta2) {
  float dist = sqrt(x*x + y*y);
  if (dist > L1 + L2 || dist < abs(L1 - L2)) return false; // unreachable

  float cos_theta2 = (x*x + y*y - L1*L1 - L2*L2) / (2 * L1 * L2);
  theta2 = acos(cos_theta2); // elbow-up solution

  float k1 = L1 + L2 * cos(theta2);
  float k2 = L2 * sin(theta2);
  theta1 = atan2(y, x) - atan2(k2, k1);

  return true;
}

//function to move the arm to coordinates (x,y,z) ADDED VERTICAL MOTION
void moveTo(float x, float y, float z, float L1, float L2) {
  float t1, t2;
  if (!inverseKinematics(x, y, L1, L2, t1, t2)) {
    return;
  }

  // Convert radians to degrees
  float deg1 = t1 * 180.0 / PI;
  float deg2 = t2 * 180.0 / PI;

  // Convert to steps
  long s1 = degToSteps(deg1);
  long s2 = degToSteps(deg2);
  long s3 = zToSteps(z);

  moveMotors(s1 - motor1_position, s2 - motor2_position, s3 - motor3_position);  // delta movement
  Serial.print("deg1: "); Serial.print (deg1); Serial.println (" degrees");
  Serial.print("deg2: "); Serial.print (deg2); Serial.println (" degrees");
}

// Custom function to move all motors concurrently coordinating the steps.
//The function receives the target values for each motor
void moveMotors(long s1, long s2, long s3) {
 long steps1 = abs(s1); //This function uses the absolute value to calculate how many steps at a time should move each motor
 long steps2 = abs(s2);
 long steps3 = abs(s3);
//These three variables will have the direction of rotation
 int dir1;
 int dir2;
 int dir3;
 //If the target is positive, the direction variable is 1
//If the target is negative, the direction variable is -1
 if (s1 >= 0) {
 dir1 = 1;
 } else {
 dir1 = -1;
 }
 if (s2 >= 0) {
 dir2 = 1;
 } else {
 dir2 = -1;
 }
 if (s3 >= 0) {
 dir3 = 1;
 } else {
 dir3 = -1;
 }
//To coordinate the movement of the motors we need to identify the largest target (absolute value)
//This line of code identifies the largest value of steps2, and steps 3. Then it identifies the largest value of that value and steps 1.
 long maxSteps = max(steps1, max(steps2, steps3));
//This part of the code is what coordinates the motion to make it smooth and at the same time.
//The for loop will execute the number of times indicated by the maximum target value.
//For each motor, divide the target number of steps for the motor by the max target.
 long stepCounter1=0;
 long stepCounter2=0;
 long stepCounter3=0;
 for (int i = 0; i < maxSteps; i++) {
  
 if ((i * steps1) / maxSteps > stepCounter1) {
      // motor1 position check
      motor1.step(dir1);
      motor1_position += dir1;
      stepCounter1++;
    }

 if ((i * steps2) / maxSteps > stepCounter2) {
    // motor1 position check
      motor2.step(dir2);
      motor2_position += dir2;
      stepCounter2++;
    }

 if ((i * steps3) / maxSteps > stepCounter3) {
    long next_position = motor3_position + dir3;
    if (next_position >= verticalMinSteps && next_position <= verticalMaxSteps) {
      motor3.step(dir3);
      motor3_position = next_position;
    }
      stepCounter3++;
 }
 delay(2); 
 }
}

long degToSteps(float degrees) {
  return (long)((degrees / 360.0) * stepsPerRevolution);
}

long zToSteps(float z) {
  const float cmPerRev = 8.85; //distance the gear rack moves on one full rotation of the pinion gear
  const float stepsPerCm = stepsPerRevolution / cmPerRev;
  return (long)(z*stepsPerCm);
}
