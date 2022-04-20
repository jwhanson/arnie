void home(){
  servo0.write(90);
  servo1.write(90);
  servo2.write(60);
  servo3.write(90);
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
