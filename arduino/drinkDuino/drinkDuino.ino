#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#define relay1 4 //arduino digital pin
#define relay2 5 //arduino digital pin
#define relay3 6 //arduino digital pin
#define relay4 7 //arduino digital pin
bool done = false; //are we done dispensing the drink?

/* ROS Setup */
ros::NodeHandle nh; //start a ROS node
void messageCb(const std_msgs::UInt16& toggle_msg){
  done = false;
  dispense(toggle_msg.data);
}
ros::Subscriber<std_msgs::UInt16> sub("dispense_drink", messageCb);
std_msgs::Bool srved_msg;
ros::Publisher served("served", &srved_msg);
/* End ROS Setup */

void setup() {
  pinMode(relay1,OUTPUT);   pinMode(relay2,OUTPUT);   pinMode(relay3,OUTPUT);   pinMode(relay4,OUTPUT);
  digitalWrite(relay1,HIGH); digitalWrite(relay2,HIGH); digitalWrite(relay3,HIGH); digitalWrite(relay4,HIGH);
  nh.initNode();
  nh.advertise(served);
  nh.subscribe(sub);
} //end void setup()

void loop() {
  srved_msg.data = done;
  served.publish( &srved_msg );
  nh.spinOnce();
  delay(500);
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
    done = true;
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
    done = true;
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
    done = true;
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
    done = true;
  }
  else{done = false;}
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
