/* drinkDuino.ino
 *
 * Author:
 * Kirk Boyd
 * 
 *
 * Description:
 * This is the code to run on the Arduino Uno attached to the back of Arnie.
 * It is responsible for connecting to the ROS master and properly timing 
 * and activating the relays which control the drink valves.
 */

#include <ros.h>
// pull in important message info for rosserial library Arduino compatibility
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#define relay1 4 //arduino digital pin
#define relay2 5 //arduino digital pin
#define relay3 6 //arduino digital pin
#define relay4 7 //arduino digital pin
bool placed = false; //is the cup on the platform?
bool prevDone = false; //I think this was going to be an extra check for something
// but I forgot lol -KB
bool done = false; //are we done dispensing the drink?
int drink = 0; //initialize drink variable

/* ROS Setup */
ros::NodeHandle nh; //start a ROS node

/* "order" topic */
void orderCb(const std_msgs::UInt16& order_msg){ // callback function for "order" subscriber
  done = false; // obviously not done because function was just called, so make sure here
  drink = order_msg.data; // store drink value

}
ros::Subscriber<std_msgs::UInt16> sub1("order", orderCb); //create subscriber for "order" topic

/* "placed" topic */
void placedCb(const std_msgs::Bool& placed_msg){ //callback function for "placed" subscriber
  placed = placed_msg.data; //store the boolean received from the placed topic
  if (placed && drink != 0 && !done){  // check that this is the right scenario to dispense
    delay(2000); // wait a moment before dispensing to prevent spillage
    dispense(drink); // pass order data to to relay interpretation function
    done = true; // store the fact that the drink was dispense so it does not run twice just in case
  }
}
ros::Subscriber<std_msgs::Bool> sub2("placed", placedCb); //create subscriber for "placed" topic

/* "served" topic */
std_msgs::Bool srved_msg; 
ros::Publisher served("served", &srved_msg);

/* "flushed" topic */

/* "rst" topic */ //MAY NEED TO BE MODIFIED. Copied format from leArm rst pub/sub
std_msgs::Bool rst_msg
ros::Publisher rst("rst", &rst_msg);
ros::Subscriber<std_msgs::Bool> sub_rst("rst", rstCb);

/* End ROS Setup */

void setup() {
  // initialize relay pins
  pinMode(relay1,OUTPUT);
  pinMode(relay2,OUTPUT);
  pinMode(relay3,OUTPUT);
  pinMode(relay4,OUTPUT);
  // ensure all relays are off
  // based on our hardware, "HIGH" or "1" keeps relays closed
  // this could be inverted a bunch of ways but it does not matter here
  digitalWrite(relay1,HIGH); 
  digitalWrite(relay2,HIGH); 
  digitalWrite(relay3,HIGH); 
  digitalWrite(relay4,HIGH);
  nh.initNode(); // start Arduino ROS node
  nh.advertise(served); // inform ROS master that "served" topic will be published to from this node
  nh.subscribe(sub1); // subscribe to "order" topic
  nh.subscribe(sub2); // subscribe to "placed" topic
} //end void setup()

void loop() { // main loop, runs forever while powered
  nh.spinOnce(); // start spinning for ROS
  delay(5);
//  relayPulse(10000);
//  digitalWrite(relay2,HIGH);
//  delay(1000);
//  digitalWrite(relay2,LOW);
//  delay(2000);
//  digitalWrite(relay2,HIGH);
//  delay(100000000000);
} //end void loop()

void dispense(int drinkNum){ 
  //this function will eventually dispense drinks at the proper ratios by 
  //sending timed relay signals based on a "drink number" input
  //but for now it just does some random timed sequences for testing
  if(drinkNum==1){
    digitalWrite(relay1,LOW);
    digitalWrite(relay2,LOW);
    digitalWrite(relay3,LOW);
    digitalWrite(relay4,LOW);
    delay(200);
    digitalWrite(relay1,HIGH);
    digitalWrite(relay2,HIGH);
    digitalWrite(relay3,HIGH);
    digitalWrite(relay4,HIGH);
    delay(200);
    digitalWrite(relay1,LOW);
    digitalWrite(relay2,LOW);
    digitalWrite(relay3,LOW);
    digitalWrite(relay4,LOW);
    delay(200);
    digitalWrite(relay1,HIGH);
    digitalWrite(relay2,HIGH);
    digitalWrite(relay3,HIGH);
    digitalWrite(relay4,HIGH);
    delay(200);
    digitalWrite(relay1,LOW);
    digitalWrite(relay2,LOW);
    digitalWrite(relay3,LOW);
    digitalWrite(relay4,LOW);
    delay(200);
    digitalWrite(relay1,HIGH);
    digitalWrite(relay2,HIGH);
    digitalWrite(relay3,HIGH);
    digitalWrite(relay4,HIGH);
    delay(200);
    srved_msg.data = true; // update actual ROS message variable
    served.publish( &srved_msg ); // publish to served topic
    //done = true;
  }
  else if(drinkNum == 2){
    digitalWrite(relay1,LOW);
    digitalWrite(relay2,LOW);
    digitalWrite(relay3,HIGH);
    digitalWrite(relay4,HIGH);
    delay(1000);
    digitalWrite(relay1,HIGH);
    digitalWrite(relay2,HIGH);
    digitalWrite(relay3,LOW);
    digitalWrite(relay4,LOW);
    delay(1000);
    digitalWrite(relay1,LOW);
    digitalWrite(relay2,LOW);
    digitalWrite(relay3,HIGH);
    digitalWrite(relay4,HIGH);
    delay(1000);
    digitalWrite(relay1,HIGH);
    digitalWrite(relay2,HIGH);
    digitalWrite(relay3,LOW);
    digitalWrite(relay4,LOW);
    delay(1000);
    srved_msg.data = true; // update actual ROS message variable
    served.publish( &srved_msg ); // publish to served topic
//    done = true;
  }
  else if(drinkNum == 3){ 
    digitalWrite(relay1,LOW);
    delay(500);
    digitalWrite(relay1,HIGH);
    delay(500);
    digitalWrite(relay2,LOW);
    delay(500);
    digitalWrite(relay2,HIGH);
    delay(500);
    digitalWrite(relay3,LOW);
    delay(500);
    digitalWrite(relay3,HIGH);
    delay(500);
    digitalWrite(relay4,LOW);
    delay(500);
    digitalWrite(relay4,HIGH);
    delay(500);
    srved_msg.data = true; // update actual ROS message variable
    served.publish( &srved_msg ); // publish to served topic
//    done = true;
  }
    else if(drinkNum == 4){
    digitalWrite(relay1,LOW);
    delay(500);
    digitalWrite(relay2,LOW);
    digitalWrite(relay1,HIGH);
    delay(500);
    digitalWrite(relay3,LOW);
    digitalWrite(relay2,HIGH);    
    delay(500);
    digitalWrite(relay4,LOW);
    digitalWrite(relay3,HIGH);
    delay(500);
    digitalWrite(relay4,HIGH);
    delay(500);
//    done = true;
    srved_msg.data = true; // update actual ROS message variable
    served.publish( &srved_msg ); // publish to served topic
  }
  else{
    srved_msg.data = false;// update actual ROS message variable
    served.publish( &srved_msg ); // publish to served topic
//    done = false;
  }
  // set servos back to closed no matter what at the end of the logic 
  // so they do not overheat or leak
  digitalWrite(relay1,HIGH); 
  digitalWrite(relay2,HIGH); 
  digitalWrite(relay3,HIGH);
  digitalWrite(relay4,HIGH);
  
}

void relayPulse(int wait){
  // simple relay test function, flashes all on then off 
  // at time specified by "wait" in ms
  digitalWrite(relay1,LOW);
  digitalWrite(relay2,LOW);
  digitalWrite(relay3,LOW);
  digitalWrite(relay4,LOW);
  delay(wait);
  digitalWrite(relay1,HIGH);
  digitalWrite(relay2,HIGH);
  digitalWrite(relay3,HIGH);
  digitalWrite(relay4,HIGH);
  delay(wait);
}
