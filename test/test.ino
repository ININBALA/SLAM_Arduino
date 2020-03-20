// Test MD03a / Pololu motor with encoder
// speed control (PI), V & I display
// Credits:
//   Dallaby   http://letsmakerobots.com/node/19558#comment-49685
//   Bill Porter  http://www.billporter.info/?p=286
//   bobbyorr (nice connection diagram) http://forum.pololu.com/viewtopic.php?f=15&t=1923

// 單輪測試程式 測試pwm, rpm, PID值, 腳位
#define InA1            6                      // INA motor pin
#define InB1            7                      // INB motor pin
#define encodPinA1      30                       // encoder A pin
#define encodPinB1      18                      // encoder B pin

#define CURRENT_LIMIT   1000                     // high current warning
#define LOW_BAT         10000                   // low bat warning
#define LOOPTIME        100                     // PID loop time
#define NUMREADINGS     10                      // samples for Amp average

int readings[NUMREADINGS];
unsigned long lastMilli = 0;                    // loop timing
unsigned long lastMilliPrint = 0;               // loop timing
int speed_req = 0;                            // speed (Set Point)
int speed_act = 0;                              // speed (actual value)
int PWM_val = 0;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
int voltage = 0;                                // in mV
int current = 0;                                // in mA
volatile long count = 0;                        // rev counter
float Kp =    4;                                // PID proportional control Gain
float Kd =    0;                                // PID Derivitave control gain


void setup() {
 analogReference(EXTERNAL);                            // Current external ref is 3.3V
 Serial.begin(115600);
 pinMode(InA1, OUTPUT);
 pinMode(InB1, OUTPUT);
 pinMode(encodPinA1, INPUT);
 pinMode(encodPinB1, INPUT);
 attachInterrupt(5, rencoder, CHANGE);
 analogWrite(InA1, PWM_val);
 analogWrite(InB1, LOW);
}

void loop() {
 getParam();                                                                 // check keyboard ao3
 if((millis()-lastMilli) >= LOOPTIME)   {                                    // enter tmed loop
   lastMilli = millis();
   getMotorData();                                                           // calculate speed, volts and Amps
   PWM_val= updatePid(PWM_val, speed_req, speed_act);                        // compute PWM value
   analogWrite(InB1, PWM_val);                                               // send PWM to motor
 }
 printMotorInfo();                                                           // display data
}

void getMotorData()  {                                                        // calculate speed, volts and Amps
static long countAnt = 0;                                                   // last count
 speed_act = ((count - countAnt)*(60*(1000/LOOPTIME)))/(780);          // 16 pulses X 29 gear ratio = 464 counts per output shaft rev
 countAnt = count;                  
}

int updatePid(int command, int targetValue, int currentValue)   {             // compute PWM value
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
   Serial.print("SP:");             Serial.print(speed_req);  
   Serial.print("  RPM:");          Serial.print(speed_act);
   Serial.print("  PWM:");          Serial.print(PWM_val); 
   Serial.println(); 
 }
}
  void rencoder() {
    if (digitalRead(encodPinB1) == HIGH) {
      if (digitalRead(encodPinA1) == LOW) {
        count++;
      }
      else {
        count--;
      }
    }
    else {
      if (digitalRead(encodPinA1) == LOW) {
        count--;
      }
      else {
        count++;
      }
    }
  }
int getParam()  {
char param, cmd;
 if(!Serial.available())    return 0;
 delay(10);                  
 param = Serial.read();                              // get parameter byte
 if(!Serial.available())    return 0;
 cmd = Serial.read();                                // get command byte
 Serial.flush();
 switch (param) {
   case 'v':                                         // adjust speed
     if(cmd=='+')  {
       speed_req += 20;
       if(speed_req>400)   speed_req=400;
     }
     if(cmd=='-')    {
       speed_req -= 20;
       if(speed_req<0)   speed_req=0;
     }
     break;
   case 's':                                        // adjust direction
     if(cmd=='+'){
       digitalWrite(InA1, LOW);
       digitalWrite(InB1, HIGH);
     }
     if(cmd=='-')   {
       digitalWrite(InA1, HIGH);
       digitalWrite(InB1, LOW);
     }
     break;
   case 'o':                                        // user should type "oo"
     digitalWrite(InA1, LOW);
     digitalWrite(InB1, LOW);
     speed_req = 0;
     break;
   default:
     Serial.println("???");
   }
}
