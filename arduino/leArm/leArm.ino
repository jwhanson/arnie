/* leArm.ino
 *
 * Author:
 * Kirk Boyd
 *
 * Description:
 * This is the code to run on the DFRObot Arduino Romeo in the front of Arnie.
 * The Romeo controls the servos in the LeArm as well as reads both switched.
 */
#include <ros.h>
// pull in important message info for rosserial library Arduino compatibility
#include <std_msgs/String.h> 
#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <Servo.h>
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

//create servo objects for each servo for the Servo library
Servo servo0; Servo servo1; Servo servo2; Servo servo3;

/* Smoothing Variables */
int goal0 = 90;       int goal1 = 90;       int goal2=90;         int goal3=90;
float goal0smooth=90; float goal1smooth=90; float goal2smooth=90; float goal3smooth=90;
float goal0prev=90;   float goal1prev=90;   float goal2prev=90;   float goal3prev=90;
float ratio1 = 0.03; //ratio of goal position
float ratio2 = 0.97; //ratio of previous position
/* End Smoothing Variables */

/* State and Timing Variables */
bool orderUp = false; //whether we have the order and can start moving
bool isPlaced = false; //has the cup been successfully placed on the platform?
bool isReset = false;
bool served = false; //are the valves done dispensing?
bool sw1State;  bool sw2State; //stores whether switch is depressed or not
bool prevPlaced = false;
bool gotCup = false; // stays true after end effector switch has been triggered to avoid bugs if cup shifts and releases the switch
bool state_setup_flag; // checks whether timer has been started
bool flushed = false; // one time check to ensure tubes are filled with liquid
bool rstNow; //if this is true a reset needs to happen and then it should get switched back as soon as that is done
int goalNum = 0; // to track which state the main loop is in
int prevGoal = 69; // to check if goalNum changed at end of each loop
int moveNum = 0; // this is the index for which move in the goals array we are on
float timeStamp; // records the start of a timer to time moves
float waitTime = 2000; //ms
int goals[][3] = { {90,   90,   60}, //0
                   {0,    50,   140}, //1
                   {90,   120,  135}, //2
                   {180,  70,   180}, //3
                   {180,  20,   177}, //4
                   {180,  37,   177}, //5
                   {180,  80,   180}, //6
                   {0,    50,   140}  }; //7
/* End State and Timing Variables */

/* ROS Setup */
ros::NodeHandle nh;
/* "order" topic */
void orderCb(const std_msgs::UInt16& order_msg){
  if (order_msg.data != 0){ // if an order message is found in the topic
    orderUp = true; // we can now move the arm
  }
  else{
    orderUp = false; // we cannot move the arm
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
/* "rst" topic */
void rstCb(const std_msgs::Bool& rst_msg){
  if (rst_msg.data == true){    
    orderUp = false;
    served = false;
    isPlaced = false;
    rstNow = true;
  }
}
std_msgs::Bool rst_msg;
ros::Publisher rst("rst", &rst_msg);
ros::Subscriber<std_msgs::Bool> sub_rst("rst", rstCb);

/* "moveNum" topic */
std_msgs::UInt16 moveNum_msg;
ros::Publisher moveNum_pub("moveNum", &moveNum_msg);
/* End ROS Setup */





void setup() {
  pinMode(sw1, INPUT); pinMode(sw2, INPUT); //tell arduino the switches are inputs
  pinMode(LED_BUILTIN,OUTPUT); // for debug
  servo0.attach(servo0pin); servo1.attach(servo1pin); servo2.attach(servo2pin); servo3.attach(servo3pin);
  nh.initNode(); // start ROS node
  nh.advertise(placed); // tell ROS master that we will publish to the "placed" topic
  nh.advertise(status); // tell ROS master that we will publish to the "status" topic
  nh.advertise(rst);
  nh.advertise(moveNum_pub);
  nh.subscribe(sub1); // subscribe to the "order" topic
  nh.subscribe(sub2); // subscribe to the "placed" topic
  nh.subscribe(sub_rst);
  home();
  delay(1500); //wait 1.5 seconds to make sure servos are home
}// end void setup()



void reset(){
  rstNow = true;
  orderUp = false;
  gotCup = false;
  isPlaced = false;
  served = false;
  goalNum = 0;
  moveNum = 0;

}


void loop() {
    sw1State = !digitalRead(sw1); //buttons read high when pressed, so we invert them 
    sw2State = !digitalRead(sw2); //which makes pressed = true and not pressed = false
    if( sw1State ){
        if( orderUp ){
            gotCup = true; // this should stay true from the first time the cup is registered in the gripper and an order is placed until the order is completed as a failsafe against losing contact with the switch
        }
    }
//    else if( !sw1State && goalNum == 6 ){
//      gotCup = false;
//    }
    if( sw2State ){
        if( gotCup ){
            isPlaced = true; // this should stay true from the first time the cup touches the stand in order to prevent jitter or weirdness if the cup moves and the switch does not stay depressed
        }
    }
//    prevGoal = goalNum; // to check if we moved at the end
    
  /*********** FSM **********/
    if( goalNum == 0 ){ /* Initial Ready State with Arm at Home and No Order */
        // STATE CONTENT
        smoothWrite(goals[moveNum]); // apply smoothing and write to servos to go home
        if(state_setup_flag){
            timeStamp = millis();
            state_setup_flag = false;
        }
        if( millis() >= timeStamp + waitTime){       
            if( !flushed ){goalNum = 1;}  // STATE TRANSITION - if not flushed go to flush sequence
            else if( orderUp && flushed ){ // otherwise once order is in, move on to receive cup
                goalNum = 2;
                moveNum = 1;
                state_setup_flag = true;
            } 
        }
    }
    else if( goalNum == 1 ){ /* Flush Sequence */
        smoothWrite(goals[moveNum]);                            // STATE CONTENT: execute flush sequence. This might have to write something to a new topic on the other arduino
        flushed = true;
        if( orderUp && flushed){ 
            if(state_setup_flag){
                  timeStamp = millis();
                  state_setup_flag = false;
            }
            if( millis() >= timeStamp + waitTime*2 ){
                goalNum = 2;
                moveNum = 1;
                state_setup_flag = true;
            }
        } 
    }
    else if( goalNum == 2 ){/* Wait For Cup */
        smoothWrite(goals[moveNum]);
        if( gotCup ){ // STATE TRANSITION - cup detected, wait before moving
            if(state_setup_flag){
                timeStamp = millis();
                state_setup_flag = false;
            }
            if( millis() >= timeStamp + waitTime ){
                goalNum = 3;
                moveNum = 2;
                state_setup_flag = true;
            }
        }
    }
    else if( goalNum == 3 ){ /* Cup Transfer Sequence */
        if(state_setup_flag){         // STATE CONTENT: if sequence has just moved to this state, the flag will be true and the timer will begin counting once.
            timeStamp = millis();
            state_setup_flag = false;   // now the flag is false, so we will not begin counting until the next step in the sequence.
        }
        if( millis() >= timeStamp + waitTime ){
            if( !isPlaced ){
                moveNum += 1;
            }
            state_setup_flag = true;
        }
        smoothWrite(goals[moveNum]);  // write the smoothed values to the servo
        if( isPlaced ){goalNum = 4;}  // STATE TRANSITION
    }

    else if( goalNum == 4){ /* Cup Hold On Plate */
                                      // STATE CONTENT /* Do Nothing for Now */
        if( served ){ // STATE TRANSITION
          if(state_setup_flag){
            timeStamp = millis();
            state_setup_flag = false;
          }
          if( millis() >= timeStamp + waitTime ){
            goalNum = 5;
            state_setup_flag = true;
          }
          
        }    
    }
    else if( goalNum == 5 ){ /* Move Cup Back to User */  
        if(state_setup_flag){         // STATE CONTENT: if sequence has just moved to this state, the flag will be true and the timer will begin counting once.
            timeStamp = millis();
            state_setup_flag = false;   // now the flag is false, so we will not begin counting until the next step in the sequence.
        }
        if( millis() >= timeStamp + waitTime ){ //step forward in goal array for next movement
            if(moveNum < 7){ moveNum += 1; }
            else{ moveNum = 7; }
            state_setup_flag = true;
        }
        smoothWrite(goals[moveNum]);  // write the smoothed values to the servo
        if( moveNum == 7 ){ // STATE TRANSITION
            if( state_setup_flag ){
                timeStamp = millis();
                state_setup_flag = false;
            }
            if( millis() >= timeStamp + waitTime ){
                goalNum = 6;
                state_setup_flag = true;
            }
        }
    }
    else if( goalNum == 6 ){ /* Final State */
        if(state_setup_flag){                              // STATE CONTENT /* Do nothing for now*/
            timeStamp = millis();
            state_setup_flag = false;
        }
        if( millis() >= timeStamp + waitTime ){
            if( !sw1State ){
              gotCup = false;
            }
            state_setup_flag = true;
            if( !gotCup ){   // STATE TRANSITION: reset everything if we got a reset message from ROS
                reset(); 
            }
        }
    }        
    if (isPlaced) { digitalWrite(LED_BUILTIN, HIGH); } // light onboard LED to tell us platform switch was triggered
    else{ digitalWrite(LED_BUILTIN, LOW);}
    if (prevPlaced != isPlaced){      // if the message has changed
        placed_msg.data = isPlaced;     // update actual ROS message variable
        placed.publish( &placed_msg);   // and publish it to the "status" topic
        prevPlaced = isPlaced;
    }
    if(prevGoal != goalNum){           // if the message has changed
        status_msg.data = goalNum;      // update actual ROS message variable
        status.publish( &status_msg );  // and publish it to the "status" topic
        prevGoal = goalNum;
    }
    if(isReset != rstNow){
        rst_msg.data = rstNow;
        rst.publish(&rst_msg);
    }
    moveNum_msg.data = moveNum;
    moveNum_pub.publish(&moveNum_msg);
    nh.spinOnce();
    delay(15);
} //end void loop()
