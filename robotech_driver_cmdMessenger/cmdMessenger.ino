void setupMessenger(){
  Serial.begin(115200); 
  cmdMessenger.printLfCr();  
  attachCommandCallbacks();
}


void sendMessage(float y) 
{

    cmdMessenger.sendCmdStart(actual_rpm);
    cmdMessenger.sendCmdArg(actual_rpm_right);
    cmdMessenger.sendCmdArg(actual_rpm_left); 
    cmdMessenger.sendCmdArg(y,3);       // millis to seconds
    cmdMessenger.sendCmdArg(y,3); 
    cmdMessenger.sendCmdEnd();
}
