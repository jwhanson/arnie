#define relay1 4 //arduino digital pin
#define relay2 5 //arduino digital pin
#define relay3 6 //arduino digital pin
#define relay4 7 //arduino digital pin

void setup() {
  pinMode(relay1,OUTPUT);   pinMode(relay2,OUTPUT);   pinMode(relay3,OUTPUT);   pinMode(relay4,OUTPUT);
  digitalWrite(relay1,LOW); digitalWrite(relay2,LOW); digitalWrite(relay3,LOW); digitalWrite(relay4,LOW);
}

void loop() {
  relayPulse(1000);
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
