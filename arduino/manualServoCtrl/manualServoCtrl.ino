/* manualServoCtrl.ino
 *
 * Author:
 * Kirk Boyd
 * 
 *
 * Description:
 * This is just a debug sketch to run on the DFRobot Romeo board which controls
 * the LeArm. It allows manual control of the servos in the arm just for positioning
 * and testing. This uses an old control scheme, however and is probably no longer useful.
 */

#include <Servo.h>
#include <Ramp.h>
#define servo0pin 3
#define servo1pin 5
#define servo2pin 6
#define servo3pin 9
#define relay1 10
#define relay2 11
#define relay3 12
#define relay4 13

// create servo objects
Servo servo0;
Servo servo1;
Servo servo2;
Servo servo3;

// stepping variables
int step0;
int step1;
int step2;
int step3;
int pos0;
int pos1;
int pos2;
int pos3;
int gol0;
int gol1;
int gol2;
int gol3;
int d0;
int d1;
int d2;
int d3;
int servo3level;

int getCup[] = {0,40,180,40}; // array to store a good 
// position for receiving the cup from the user. Also deprecated.

long t = 2500; // time for each motion to be completed
int dT = 50; // time step
int N; // = 50; //t/dT;
int pos = 0;    // variable to store the servo position

void setup() {
  Serial.begin(9600);
  pinMode(relay1,OUTPUT);
  pinMode(relay2,OUTPUT);
  pinMode(relay3,OUTPUT);
  pinMode(relay4,OUTPUT);
  servo0.attach(servo0pin);
  servo1.attach(servo1pin);
  servo2.attach(servo2pin);
  servo3.attach(servo3pin);
  digitalWrite(relay1,LOW);
  digitalWrite(relay2,LOW);
  digitalWrite(relay3,LOW);
  digitalWrite(relay4,LOW);
  home();
  Serial.println("Home");
  delay(1500);
}



void loop() {
  servo0.write(0);
//  delay(2000);
}

void moveLvl(int gol0, int gol1, int gol2, int gol3){  
  // this program i
  N = t/dT;
  pos0 = servo0.read(); //90;
  pos1 = servo1.read(); //90;
  pos2 = servo2.read(); //90;
  pos3 = servo3.read(); //90;
  d0 = gol0 - pos0;
  d1 = gol1 - pos1;
  d2 = gol2 - pos2;
  d3 = gol3 - pos3;
  step0 = d0/N;
  step1 = d1/N;
  step2 = d2/N;
  step3 = d3/N;
    for (int i=0; i<N; i++){
      servo0.write(i*step0 + pos0);
      servo3level = (i*step1+pos1) + (180 - (i*step2+pos2));
      servo3.write(servo3level);
      servo1.write(i*step1 + pos1);
      servo2.write(i*step2 + pos2);
      delay(dT);
    }
}

void sweep(int goal){
  // this function is an adaptation of the Arduino Servo.h library example "sweep".
  // it just runs through every position on every servo in a loop.
    for (pos = 0; pos <= goal; pos += 1) { // goes from 0 degrees to 180deg in steps of 1deg
    servo0.write(pos);  // tell servo to go to position in variable 'pos'
    servo1.write(pos);       
    servo2.write(pos);       
    servo3.write(pos);       
    delay(2);  
    }// waits 15 ms for the servo to reach the position
}

void printServos(){
  // this was for very early debugging to read the pots in the servos.
  // it was not very reliable so this function is probably not worth using.
//      Serial.print("joint0 desired: ");
//      Serial.print(i*step0 + pos0);
//      Serial.print("\t | joint1 desired: ");
//      Serial.print(i*step1 + pos1);
//      Serial.print("\t | joint2 desired: ");
//      Serial.print(i*step2 + pos2);
//      Serial.print("\t | joint3 desired: ");
//      Serial.println(servo3level);
      Serial.print("joint0 reading: ");
      Serial.print(servo0.read());
      Serial.print("\t | joint1 reading: ");
      Serial.print(servo1.read());
      Serial.print("\t | joint2 reading: ");
      Serial.print(servo2.read());
      Serial.print("\t | joint3 reading: ");
      Serial.println(servo3.read());
}
