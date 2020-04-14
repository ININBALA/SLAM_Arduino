#include <ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>

#define InA1            4                      // INA motor pin
#define InB1            5  
#define InA2            6
#define InB2            7
#define InA3            2
#define InB3            3
// INB motor pin
//#define PWM1            6                       // PWM motor pin
#define encodPinA1      31                      // encoder A pin
#define encodPinB1      21                       // encoder B pin
#define encodPinA2      20                      // encoder A pin
#define encodPinB2      30
#define encodPinA3      19                      // encoder A pin
#define encodPinB3      33
//#define Vpin            0                       // battery monitoring analog pin
//#define Apin            1                       // motor current monitoring analog pin

#define CURRENT_LIMIT   1000                     // high current warning
#define LOW_BAT         10000                   // low bat warning
#define LOOPTIME        100                     // PID loop time
#define NUMREADINGS     10                      // samples for Amp average
ros::NodeHandle nh;

int readings[NUMREADINGS];
unsigned long lastMilli = 0;                    // loop timing
unsigned long lastMilliPrint = 0;               // loop timing
int speed_req1 = 20;                            // speed (Set Point)
int speed_act1 = 0;                              // speed (actual value)
int PWM_val1 = 0;                    // (25% = 64; 50% = 127; 75% = 191; 100% = 255)

int speed_req2 = 20;                            // speed (Set Point)
int speed_act2 = 0;                              // speed (actual value)
int PWM_val2 = 0;   
int speed_req3 = 20;                            // speed (Set Point)
int speed_act3 = 0;                              // speed (actual value)
int PWM_val3 = 0;   
//int voltage = 0;                                // in mV
//int current = 0;                                // in mA
volatile long count1 = 0;  
volatile long count2 = 0;
volatile long count3 = 0;// rev counter
float Kp =   .4;                                // PID proportional control Gain
float Kd =    1;                                // PID Derivitave control gain

geometry_msgs::Vector3Stamped rpm_msg;
ros::Publisher rpm_pub("rpm", &rpm_msg);
void setup() {
 analogReference(EXTERNAL);                            // Current external ref is 3.3V
 Serial.begin(115200);
 pinMode(InA1, OUTPUT);
 pinMode(InB1, OUTPUT);
 pinMode(InA2, OUTPUT);
 pinMode(InB2, OUTPUT);
 pinMode(InA3, OUTPUT);
 pinMode(InB3, OUTPUT);
 //pinMode(PWM1, OUTPUT);
 pinMode(encodPinA1, INPUT);
 pinMode(encodPinB1, INPUT);
 pinMode(encodPinA2, INPUT);
 pinMode(encodPinB2, INPUT);
 pinMode(encodPinA3, INPUT);
 pinMode(encodPinB3, INPUT);
 digitalWrite(encodPinA1, HIGH);                      // turn on pullup resistor
 digitalWrite(encodPinB1, HIGH);
 digitalWrite(encodPinA2, HIGH);                      // turn on pullup resistor
 digitalWrite(encodPinB2, HIGH);
 digitalWrite(encodPinA3, HIGH);                      // turn on pullup resistor
 digitalWrite(encodPinB3, HIGH);
 attachInterrupt(2, rencoder1, FALLING);
 attachInterrupt(3, rencoder2, FALLING);
 attachInterrupt(4, rencoder3, FALLING);
 for(int i=0; i<NUMREADINGS; i++)   readings[i] = 0;  // initialize readings to 0

// analogWrite(PWM1, PWM_l);
 analogWrite(InA1, LOW);
 analogWrite(InB1, PWM_val1);
 analogWrite(InA2, LOW);
 analogWrite(InB2, PWM_val2);
 analogWrite(InA3, LOW);
 analogWrite(InB3, PWM_val3);

 nh.initNode();
 nh.advertise(rpm_pub);
 //nh.getHardware()->setBaud(57600);
}

void loop() {
 nh.spinOnce();
 //getParam();                                                                 // check keyboard
 if((millis()-lastMilli) >= LOOPTIME)   {                                    // enter tmed loop 此function會和publisher爭奪serial port 故註解
   lastMilli = millis();
   getMotorData();                                                           // calculate speed, volts and Amps
   PWM_val1= updatePid1(PWM_val1, speed_req1, speed_act1);
   PWM_val2= updatePid2(PWM_val2, speed_req2, speed_act2);  
   PWM_val3= updatePid3(PWM_val3, speed_req3, speed_act3);  // compute PWM value
   analogWrite(InB1, PWM_val1);
   analogWrite(InB2, PWM_val2);  
   analogWrite(InB3, PWM_val3);  // send PWM to motor
   publishRPM();
   //nh.spinOnce();
 }
 //printMotorInfo();                                                           // display data
}

void getMotorData()  {                                                        // calculate speed, volts and Amps
static long countAnt1 = 0;  
static long countAnt2 = 0;
static long countAnt3 = 0;// last count
 speed_act1 = ((count1 - countAnt1)*(60*(1000/LOOPTIME)))/(16*29);          // 16 pulses X 29 gear ratio = 464 counts per output shaft rev
 countAnt1 = count1; 
 speed_act2 = ((count2 - countAnt2)*(60*(1000/LOOPTIME)))/(16*29);          // 16 pulses X 29 gear ratio = 464 counts per output shaft rev
 countAnt2 = count2; 
 speed_act3 = ((count3 - countAnt3)*(60*(1000/LOOPTIME)))/(16*29);          // 16 pulses X 29 gear ratio = 464 counts per output shaft rev
 countAnt3 = count3;                  
// voltage = int(analogRead(Vpin) * 3.22 * 12.2/2.2);                          // battery voltage: mV=ADC*3300/1024, voltage divider 10K+2K
// current = int(analogRead(Apin) * 3.22 * .77 *(1000.0/132.0));               // motor current - output: 130mV per Amp
// current = digital_smooth(current, readings);                                // remove signal noise
}

int updatePid1(int command, int targetValue, int currentValue)   {             // compute PWM value
float pidTerm = 0;                                                            // PID correction
int error=0;                                  
static int last_error=0;                            
 error = abs(targetValue) - abs(currentValue);
 pidTerm = (Kp * error) + (Kd * (error - last_error));                            
 last_error = error;
 return constrain(command + int(pidTerm), 0, 255);
}

int updatePid2(int command, int targetValue, int currentValue)   {             // compute PWM value
float pidTerm = 0;                                                            // PID correction
int error=0;                                  
static int last_error=0;                            
 error = abs(targetValue) - abs(currentValue);
 pidTerm = (Kp * error) + (Kd * (error - last_error));                            
 last_error = error;
 return constrain(command + int(pidTerm), 0, 255);
}

int updatePid3(int command, int targetValue, int currentValue)   {             // compute PWM value
float pidTerm = 0;                                                            // PID correction
int error=0;                                  
static int last_error=0;                            
 error = abs(targetValue) - abs(currentValue);
 pidTerm = (Kp * error) + (Kd * (error - last_error));                            
 last_error = error;
 return constrain(command + int(pidTerm), 0, 255);
}

void printMotorInfo()  {                                                      // display data
 if((millis()-lastMilliPrint) >= 500)   {                    
   lastMilliPrint = millis();
   Serial.print("SP1:");             Serial.print(speed_req1);  
   Serial.print("  RPM1:");          Serial.print(speed_act1);
   Serial.print("  PWM1:");          Serial.print(PWM_val1);
    Serial.print("  SP2:");             Serial.print(speed_req2);  
   Serial.print("  RPM2:");          Serial.print(speed_act2);
   Serial.print("  PWM2:");          Serial.print(PWM_val2);
    Serial.print("  SP3:");             Serial.print(speed_req3);  
   Serial.print("  RPM3:");          Serial.print(speed_act3);
   Serial.print("  PWM3:");          Serial.print(PWM_val3);  
   Serial.println();
//   Serial.print("  V:");            Serial.print(float(voltage)/1000,1);
//   Serial.print("  mA:");           Serial.println(current);

//   if (current > CURRENT_LIMIT)               Serial.println("*** CURRENT_LIMIT ***");                
//   if (voltage > 1000 && voltage < LOW_BAT)   Serial.println("*** LOW_BAT ***");                
 }
}

 void rencoder1() {
    if (digitalRead(encodPinB1) == HIGH) {
      if (digitalRead(encodPinA1) == LOW) {
        count1++;
      }
      else {
        count1--;
      }
    }
    else {
      if (digitalRead(encodPinA1) == LOW) {
        count1--;
      }
      else {
        count1++;
      }
    }
  }


 void rencoder2() {
    if (digitalRead(encodPinB2) == HIGH) {
      if (digitalRead(encodPinA2) == LOW) {
        count2++;
      }
      else {
        count2--;
      }
    }
    else {
      if (digitalRead(encodPinA2) == LOW) {
        count2--;
      }
      else {
        count2++;
      }
    }
  }

 void rencoder3() {
    if (digitalRead(encodPinB3) == HIGH) {
      if (digitalRead(encodPinA3) == LOW) {
        count3++;
      }
      else {
        count3--;
      }
    }
    else {
      if (digitalRead(encodPinA3) == LOW) {
        count3--;
      }
      else {
        count3++;
      }
    }
  }



/*void rencoder()  {                                    // pulse and direction, direct port reading to save cycles
 if (PINB & 0b00000001)    count++;                // if(digitalRead(encodPinB1)==HIGH)   count ++;
 else                      count--;                // if (digitalRead(encodPinB1)==LOW)   count --;
}*/

int getParam()  {
char param1, cmd1;
 if(!Serial.available())    return 0;
 delay(10);                  
 param1 = Serial.read();                              // get parameter byte
 if(!Serial.available())    return 0;
 cmd1 = Serial.read();                                // get command byte
 Serial.flush();
 
 switch (param1) {
   case 'w':                                         // adjust speed
     if(cmd1=='+')  {
       speed_req1 += 20;
       if(speed_req1>400)   speed_req1=400;
     }
     if(cmd1=='-')    {
       speed_req1 -= 20;
       if(speed_req1<0)   speed_req1 =0;
     }
     break;
   case 's':                                        // adjust direction
     if(cmd1=='+'){
       digitalWrite(InA1, HIGH);
       digitalWrite(InB1, LOW);
     }
     if(cmd1=='-')   {
       digitalWrite(InA1, HIGH);
       digitalWrite(InB1, LOW); 
     }
     break;
   case 'x':                                        // user should type "oo"
     digitalWrite(InA1, LOW);
     digitalWrite(InB1, LOW);
     speed_req1 = 0;
     break;

   case 'e':                                         // adjust speed
     if(cmd1=='+')  {
       speed_req2 += 20;
       if(speed_req2>400)   speed_req2=400;
     }
     if(cmd1=='-')    {
       speed_req2 -= 20;
       if(speed_req2<0)   speed_req2 =0;
     }
     break;
   case 'd':                                        // adjust direction
     if(cmd1=='+'){
       digitalWrite(InA2, HIGH);
       digitalWrite(InB2, LOW);
     }
     if(cmd1=='-')   {
       digitalWrite(InA2, HIGH);
       digitalWrite(InB2, LOW); 
     }
     break;
   case 'c':                                        // user should type "oo"
     digitalWrite(InA2, LOW);
     digitalWrite(InB2, LOW);
     speed_req2 = 0;
     break;

   case 'r':                                         // adjust speed
     if(cmd1=='+')  {
       speed_req3 += 20;
       if(speed_req3>400)   speed_req3=400;
     }
     if(cmd1=='-')    {
       speed_req3 -= 20;
       if(speed_req3<0)   speed_req3 =0;
     }
     break;
   case 'f':                                        // adjust direction
     if(cmd1=='+'){
       digitalWrite(InA3, HIGH);
       digitalWrite(InB3, LOW);
     }
     if(cmd1=='-')   {
       digitalWrite(InA3, HIGH);
       digitalWrite(InB3, LOW); 
     }
     break;
   case 'v':                                        // user should type "oo"
     digitalWrite(InA3, LOW);
     digitalWrite(InB3, LOW);
     speed_req3 = 0;
     break;
   default:
     Serial.println("???");
   }
}

void publishRPM(){
  rpm_msg.header.stamp = nh.now();
  rpm_msg.vector.x = speed_act1;
  rpm_msg.vector.y = speed_act2;
  rpm_msg.vector.z = speed_act3;
  rpm_pub.publish(&rpm_msg);
  nh.spinOnce();
}
/*int digital_smooth(int value, int *data_array)  {    // remove signal noise
static int ndx=0;                                                        
static int count=0;                          
static int total=0;                          
 total -= data_array[ndx];              
 data_array[ndx] = value;                
 total += data_array[ndx];              
 ndx = (ndx+1) % NUMREADINGS;                                
 if(count < NUMREADINGS)      count++;
 return total/count;
}*/
