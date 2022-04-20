int switch;
float switch1Smoothed;
float switch1Prev;

void setup(){
   Serial.begin(115200);
   pinMode(12,INPUT_PULLUP);
}

void loop(){
  switch1 = digitalRead(12); //read switch
  switch1 = switch1 * 100;  //multipy by 100

  /* Smoothing */
  switch1Smoothed = (switch1 + 0.05) + (switch1Prev * 0.95);
  switch1Prev = switch1Smoothed;
  /* End of Smoothing */

  Serial.print(switch1);
  Serial.print(" , ");
  Serial.println(switch1Smoothed);

  delay(10);
}
