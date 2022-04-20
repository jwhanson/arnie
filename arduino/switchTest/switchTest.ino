#define sw1 2 //pin for switch on end effector
#define sw2 7 //pin for switch in receive plate
bool sw1State;  bool sw2State; //stores whether switch is depressed or not


void setup() {
  Serial.begin(9600); //for debug
  pinMode(sw1, INPUT); pinMode(sw2, INPUT); //tell arduino the switches are inputs

}

void loop() {
  sw1State = !digitalRead(sw1); //buttons read high when pressed, so we invert them 
  sw2State = !digitalRead(sw2); //which makes pressed = true and not pressed = false
  Serial.print(sw1State);
  Serial.print(" , ");
  Serial.println(sw2State);

}
