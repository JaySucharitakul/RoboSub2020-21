#include <Servo.h>
/*      
self.ThrusterLB = ThrusterDriver(2, self.board)  # left back
self.ThrusterLF = ThrusterDriver(4, self.board)  # left front
self.ThrusterRB = ThrusterDriver(3, self.board)  # right back
self.ThrusterRF = ThrusterDriver(5, self.board)  # right front
self.ThrusterBL = ThrusterDriver(6, self.board)  # back left
self.ThrusterBR = ThrusterDriver(7, self.board)  # back right
self.ThrusterFL = ThrusterDriver(8, self.board)  # front left
self.ThrusterFR = ThrusterDriver(9, self.board)  # front right
*/
        
byte LBpin = 2; //left back
byte LFpin = 3; //left front
byte RBpin = 4; //right back
byte RFpin = 5; //right front
byte BLpin = 6; //back left
byte BRpin = 7; //back right
byte FLpin = 8; //front left
byte FRpin = 9; //front right
Servo LBthruster;
Servo LFthruster;
Servo RBthruster;
Servo RFthruster;
Servo BLthruster;
Servo BRthruster;
Servo FLthruster;
Servo FRthruster;
enum state {
  standby,
  running
};
enum state = running;
void setup() {
  const char delim[2] = "-";
  const char delimalt[2] = " ";
  int thrusterpower[6];
  LBthruster.attach(LBpin);
  LFthruster.attach(LFpin);
  RBthruster.attach(RBpin);
  RFthruster.attach(RFpin);
  BLthruster.attach(BLpin);
  BRthruster.attach(BRpin);
  FLthruster.attach(FLpin);
  FRthruster.attach(FRpin);
  
  LBthruster.writeMicroseconds(1500); // send "stop" signal to ESC. Also necessary to arm the ESC.
  LFthruster.writeMicroseconds(1500); 
  RBthruster.writeMicroseconds(1500); 
  RFthruster.writeMicroseconds(1500); 
  BLthruster.writeMicroseconds(1500); 
  BRthruster.writeMicroseconds(1500); 
  FLthruster.writeMicroseconds(1500); 
  FRthruster.writeMicroseconds(1500); 
  delay(7000); // delay to allow the ESC to recognize the stopped signal.
  Serial.begin(9600);
}

void loop(){
    
    /*I want the motor data in stuff to be structured like:
     * LBpower - LFpower - RBpower - RFpower - BLpower - BRpower - FLpower - FRpower
     * 
     */
    if (Serial.available()) {
      String datain = Serial.readString();// s1 is String type variable.
      Serial.print("Received Data => ");
      Serial.println(datain);//display same received Data back in serial monitor.
      if(state == running){
        int i = 0;
        token = strtok(datain,s);
        /* walk through other tokens */
        while( token != NULL ) {
          thrusterpower[i] = int(token)
          i = i + 1;
          token = strtok(NULL,s);
        }
        delay(500);
      }
      else if(state == standby){
        token = strtok(datain,s);
        /* walk through other tokens */
        while( token != NULL ) {
          if(token == "START"){
            state = running;
          }
          token = strtok(NULL,s);
        }
      }
    }
}
