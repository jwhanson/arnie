void home(){
  servo0.write(90);
  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
}
//void homeRamp(){
//  servo0.write(ramp0.go(90,500));
//  servo1.write(ramp1.go(90,500));
//  servo2.write(ramp2.go(90,500));
//  servo3.write(ramp3.go(90,500));
//}
//void sweep(){
//    for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180deg in steps of 1deg
//    servo0.write(pos);  // tell servo to go to position in variable 'pos'
//    servo1.write(pos);       
//    servo2.write(pos);       
//    servo3.write(pos);       
//    delay(2);           // waits 15 ms for the servo to reach the position
//  }
//  delay(2000);   
//  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
//    servo0.write(pos);  // tell servo to go to position in variable 'pos'
//    servo1.write(pos);       
//    servo2.write(pos);       
//    servo3.write(pos); 
//    delay(2);           // waits 15 ms for the servo to reach the position
//  }
//  delay(2000);   
//}
void getCup_max_ext(){
  servo0.write(0);
  servo1.write(0);
  servo2.write(90);
  servo3.write(90);
}
void getCup_cute(){
  servo0.write(0);
  servo1.write(40);
  servo2.write(180);
  servo3.write(40);
}
//void getCup_cute_ramp(){
//  servo0.write(ramp0.go(0,500));
//  servo1.write(ramp1.go(40,500));
//  servo2.write(ramp2.go(180,500));
//  servo3.write(ramp3.go(40,500));
//}
void pulled_up_cntrd(){
  servo0.write(90);
  servo1.write(135);
  servo2.write(135);
  servo3.write(175);
}
//void pulled_up_cntrd_ramp(){
//  servo0.write(ramp0.go(90,500));
//  servo1.write(ramp1.go(135,500));
//  servo2.write(ramp2.go(135,500));
//  servo3.write(ramp3.go(175,500));
//}
void fill_max_ext(){
  servo0.write(180);
  servo1.write(0);
  servo2.write(90);
  servo3.write(90);
}
void fill_cute(){
  servo0.write(180);
  servo1.write(40);
  servo2.write(180);
  servo3.write(40);
}
