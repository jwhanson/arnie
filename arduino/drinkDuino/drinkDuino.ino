#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#define relay1 4 //arduino digital pin
#define relay2 5 //arduino digital pin
#define relay3 6 //arduino digital pin
#define relay4 7 //arduino digital pin
bool placed = false; //is the cup on the platform?
bool prevDone = false; //
bool done = false; //are we done dispensing the drink?
int drink = 0;

/* ROS Setup */
ros::NodeHandle nh; //start a ROS node

/* "order" topic */
void orderCb(const std_msgs::UInt16& order_msg){ //callback function for "order" subscriber
  done = false; //obviously not done because function was just called, so make sure here
  drink = order_msg.data;

}
ros::Subscriber<std_msgs::UInt16> sub1("order", orderCb); //create subscriber for "order" topic

/* "placed" topic */
void placedCb(const std_msgs::Bool& placed_msg){ //callback function for "placed" subscriber
  placed = placed_msg.data; //store the boolean received from the placed topic
  if (placed && drink != 0 && !done){  
    delay(2000);
    dispense(drink); //pass order data to to relay interpretation function
    done = true;
  }
//  else{
//    done = false;
//  }
}
ros::Subscriber<std_msgs::Bool> sub2("placed", placedCb); //create subscriber for "placed" topic

/* "served" topic */
std_msgs::Bool srved_msg; 
ros::Publisher served("served", &srved_msg);
/* End ROS Setup */

void setup() {
  pinMode(relay1,OUTPUT);   pinMode(relay2,OUTPUT);   pinMode(relay3,OUTPUT);   pinMode(relay4,OUTPUT);
  digitalWrite(relay1,HIGH); digitalWrite(relay2,HIGH); digitalWrite(relay3,HIGH); digitalWrite(relay4,HIGH);
  nh.initNode();
  nh.advertise(served);
  nh.subscribe(sub1);
  nh.subscribe(sub2);
} //end void setup()

void loop() {
//  relayPulse(500);

  nh.spinOnce();
  delay(5);
} //end void loop ()

void dispense(int drinkNum){
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
    srved_msg.data = true;
    served.publish( &srved_msg );
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
    srved_msg.data = true;
    served.publish( &srved_msg );
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
    srved_msg.data = true;
    served.publish( &srved_msg );
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
    srved_msg.data = true;
    served.publish( &srved_msg );
  }
  else{
    srved_msg.data = false;
    served.publish( &srved_msg );
//    done = false;
  }
  digitalWrite(relay1,HIGH); digitalWrite(relay2,HIGH); digitalWrite(relay3,HIGH); digitalWrite(relay4,HIGH);
  
}

void relayPulse(int wait){
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
