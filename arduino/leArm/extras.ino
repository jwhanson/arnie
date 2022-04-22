void home(){
  servo0.write(90);
  servo1.write(90);
  servo2.write(60);
  servo3.write(90);
}
void smoothWrite(int vals[3]){ //applies the smoothing filter to the array and writes to the servos
    goal3 = vals[1] + (140 - vals[2]); //calculate last servo angle to hold cup level

    goal0smooth = (vals[0] * ratio1) + (goal0prev * ratio2);
    goal1smooth = (vals[1] * ratio1) + (goal1prev * ratio2);
    goal2smooth = (vals[2] * ratio1) + (goal2prev * ratio2);
    goal3smooth = (goal3   * ratio1) + (goal3prev * ratio2);
    goal0prev = goal0smooth;
    goal1prev = goal1smooth;
    goal2prev = goal2smooth;
    goal3prev = goal3smooth;
    servo0.write(goal0smooth);
    servo1.write(goal1smooth);
    servo2.write(goal2smooth);
    servo3.write(goal3smooth);
}

void printSw(){ // print switches nicely formatted for debug
  Serial.print(sw1State);
  Serial.print(" , ");
  Serial.println(sw2State);
}
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
void pulled_up_cntrd(){
  servo0.write(90);
  servo1.write(135);
  servo2.write(135);
  servo3.write(175);
}
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
