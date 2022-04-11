#include <ros.h>
#include <std_msgs/String.h> //for compatibility with ROS
#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <Servo.h>
//#include <Ramp.h>
#define sw1 2 //pin for switch on end effector
#define sw2 7 //pin for switch in receive plate
#define servo0pin 3 //base servo pin
#define servo1pin 5 //joint 1 servo pin
#define servo2pin 6 //joint 2 servo pin
#define servo3pin 9 //joint 3 servo pin
#define relay1 10 //arduino digital pin
#define relay2 11 //arduino digital pin
#define relay3 12 //arduino digital pin
#define relay4 13 //arduino digital pin
Servo servo0; Servo servo1; Servo servo2; Servo servo3; //create servo objects for each servo for the library
int getCup[] = {0,40,180,40}; // these are good positions to receive the cup
bool orderUp = false; //whether we have the order and can start moving
bool isPlaced = false; //has the cup been successfully placed on the platform?
bool serving = false; //are we attempting to place cup and serve a drink?
bool served = false; //are the valves done dispensing?
bool sw1State;  bool sw2State; //stores whether switch is depressed or not
/* Smoothing Variables */
int goal0 = 90;       int goal1 = 90;       int goal2=90;         int goal3=90;
float goal0smooth=90; float goal1smooth=90; float goal2smooth=90; float goal3smooth=90;
float goal0prev=90;   float goal1prev=90;   float goal2prev=90;   float goal3prev=90;
float ratio1 = 0.03; //ratio of goal position
float ratio2 = 0.97; //ratio of previous position
/* End Smoothing Variables */

bool startedCounting = false;
bool countingAgain = false;
bool countingAgainAgain = false;
bool counting4;
float timer; //ms
float timer2;
float timer3;
float timer4;
float waitTime = 3000; //ms
float waitTime2 = 2000; //ms
int status_string;
int prevString;
int string_msg;

/* ROS Setup */
ros::NodeHandle nh;

/* "order" topic */
void orderCb(const std_msgs::UInt16& order_msg){
  if (order_msg.data != 0){
    orderUp = true;
  }
  else{
    orderUp = false;
  }
}
ros::Subscriber<std_msgs::UInt16> sub1("order", orderCb);

/* "served" topic */
void servedCb(const std_msgs::Bool& served_msg){
  served = served_msg.data;
}
ros::Subscriber<std_msgs::Bool> sub2("served", servedCb);

/* "placed" topic */
std_msgs::Bool placed_msg;
ros::Publisher placed("placed", &placed_msg);

/* "status" topic */
std_msgs::UInt16 status_msg;
ros::Publisher status("status", &status_msg);
/* End ROS Setup */

void setup() {
    Serial.begin(9600); //for debug
  pinMode(sw1, INPUT); pinMode(sw2, INPUT); //tell arduino the switches are inputs
  pinMode(LED_BUILTIN,OUTPUT);
  servo0.attach(servo0pin); servo1.attach(servo1pin); servo2.attach(servo2pin); servo3.attach(servo3pin);
  nh.initNode();
  nh.advertise(placed);
  nh.advertise(status);
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  home();
    Serial.println("Home");
  delay(1500); //wait 1.5 seconds to make sure servos are home
}// end void setup()


void loop() {
  sw1State = !digitalRead(sw1); //buttons read high when pressed, so we invert them 
  sw2State = !digitalRead(sw2); //which makes pressed = true and not pressed = false
  Serial.print(sw1State);
  Serial.print(" , ");
  Serial.println(sw2State);
  prevString = status_string;
  if(orderUp){
    if(!sw1State && !sw2State && !served && !isPlaced && !serving){ //if end effector and plate are not depressed
      //maybe add !orderUp******
      
      // move to "receive cup" pose
      goal0 = 0;
      goal1 = 50;
      goal2 = 140;
      isPlaced = false;
//      status_string = "Waiting for cup";
      status_string = 1;
    }
    else if(sw1State && !sw2State && !served && !startedCounting){ //if
      // wait a sec for user to put in cup and get their hand away
      timer = millis();
      serving = true;
      startedCounting = true;
      isPlaced = false;
//      status_string = "Got cup, waiting a sec";
      status_string = 2;
    }
    else if(startedCounting && timer + waitTime <= millis() && !sw2State && !countingAgain){
      // move to an intermediate position to avoid obstacles
      goal0 = 90;
      goal1 = 120;
      goal2 = 135;
      timer2 = millis();
      countingAgain = true;
      isPlaced = false;
//      status_string = "Moving to stand step 1";
      status_string = 3;
    }
    else if(startedCounting && countingAgain && !countingAgainAgain && timer + waitTime <= millis() && !sw2State && timer2 + waitTime2 <= millis()){
      // get closer to platform
      goal0 = 160;
      goal1 = 55;
      goal2 = 150;
      isPlaced = false;
      timer3 = millis();
      countingAgainAgain = true;
//      status_string = "Moving to stand step 2";
      status_string = 4;
    }
    else if(countingAgainAgain && !sw2State && timer3 + waitTime2 <= millis()){
      // place cup on platform
      goal0 = 180;
      goal1 = 20;
      goal2 = 177;
      isPlaced = false;
//      status_string = "Putting cup on stand";
      status_string = 5;
    }
    else if(sw2State && !served && serving){
      // keep cup on platform
      goal0 = 180;
      goal1 = 37;
      goal2 = 177;
      timer4 = millis();
      counting4 = true;
      isPlaced = true;
//      status_string = "Cup on stand, dispensing";
      status_string = 6;
    }
    else if(served && counting4 && timer4 + waitTime2 <= millis()){
      // pass drink back to user
      goal0 = 180;
      goal1 = 50;
      goal2 = 140;
      isPlaced = true;
//      status_string = "Moving back to user, step 1";
      status_string = 7;
    }
    else if(served){
      // pass drink back to user
      goal0 = 0;
      goal1 = 50;
      goal2 = 140;
      isPlaced = true;
//      status_string = "Cup should be at user now.";
      status_string = 8;
    }
    
    goal3 = goal1 + (140 - goal2); //keep cup level
    
    /* Smoothing */
    goal0smooth = (goal0 * ratio1) + (goal0prev * ratio2);
    goal1smooth = (goal1 * ratio1) + (goal1prev * ratio2);
    goal2smooth = (goal2 * ratio1) + (goal2prev * ratio2);
    goal3smooth = (goal3 * ratio1) + (goal3prev * ratio2);
    //  Serial.print(goal0);
    //  Serial.print(" , ");
    //  Serial.println(goal0smooth);
    goal0prev = goal0smooth;
    goal1prev = goal1smooth;
    goal2prev = goal2smooth;
    goal3prev = goal3smooth;
    /* End Smoothing */
  
    /* Write Smoothed Values to Servos */
    servo0.write(goal0smooth);
    servo1.write(goal1smooth);
    servo2.write(goal2smooth);
    servo3.write(goal3smooth);
  }
  if (isPlaced) { digitalWrite(LED_BUILTIN, HIGH);}
  else{ digitalWrite(LED_BUILTIN, LOW);}
  placed_msg.data = isPlaced;
  placed.publish( &placed_msg);
  string_msg = status_string;
  if(prevString != status_string){
    status_msg.data = string_msg;
  }
  status.publish( &status_msg );
  nh.spinOnce();
  delay(15);
} //end void loop()














void printServos(){
  Serial.print("joint0 reading: ");
  Serial.print(servo0.read());
  Serial.print("\t | joint1 reading: ");
  Serial.print(servo1.read());
  Serial.print("\t | joint2 reading: ");
  Serial.print(servo2.read());
  Serial.print("\t | joint3 reading: ");
  Serial.println(servo3.read());
}


/* GRAVEYARD */
