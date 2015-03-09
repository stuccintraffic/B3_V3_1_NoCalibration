// B3 //
// Driver's Seat //





#include  "WProgram.h"
#include  "chipKITCAN.h"
#include  "AccelStepper.h"
#define SYS_FREQ	(80000000L)
#define CAN_BUS_SPEED   500000		// CAN Speed, in bits per second
CAN    canMod1(CAN::CAN1);
uint8_t  CAN1MessageFifoArea[2 * 8 * 16];

word T_SID = 0x46A;  //Transmit data variables, user will set these variable before calling the txCAN1() to put the data on the bus. 
volatile uint8_t T_1 = 0;                          
volatile uint8_t T_2 = 0;
volatile uint8_t T_3 = 0;
volatile uint8_t T_4 = 0;
volatile uint8_t T_5 = 0;
volatile uint8_t T_6 = 0;
volatile uint8_t T_7 = 0;
volatile uint8_t T_8 = 0;

word R_SID = 0;  //Receive data variables, they get stored when the rxCAN1() function is called and only saves off the last Message in the buffer.
volatile uint8_t R_1 = 0;                          
volatile uint8_t R_2 = 0;
volatile uint8_t R_3 = 0;
volatile uint8_t R_4 = 0;
volatile uint8_t R_5 = 0;
volatile uint8_t R_6 = 0;
volatile uint8_t R_7 = 0;
volatile uint8_t R_8 = 0;

uint8_t R_1_prev = 0;
uint8_t R_2_prev = 0;
uint8_t R_4_prev = 0;
uint8_t R_5_prev = 0;
uint8_t R_6_prev = 0;
uint8_t R_7_prev = 0;

                        
uint32_t R_Filter = 0x00E;  //Receive SID that CAN 1 will pull off the CAN 1 bus
uint32_t R_Mask = 0x004;    //Receive SID Bitmask for more than one SID to get pulled off. If it is 0xFFF, only R_Filter will get pulled off.
                            //If an R_Mask bit is zero, then the filter will not look at that bit. Example: if R_Filter is 0x001 and R_Mask is 0xFFE (all 1's but last bit)
                            //CAN 1 will pull both 0x001 & 0x000 messages off the bus. 


static volatile bool isCAN1MsgReceived = false; //could change inside CAN 1 interupt so "volatile" required to tell processor to remember it
void initCan1(void);
void doCan1Interrupt();
void txCAN1(uint32_t rxnode);
void rxCAN1(void);

int max_deviation = 2;         //set max allowable deviation from pot1 to pot2 before 
int Zfront_diff;             //encoder differential at front z axis
int Zrear_diff;              //encoder differential at rear z axis
int Zfront_int;              //front z value at node (integer format)
int Zrear_int;               //rear z value at node (integer formaat)

long X_position;
long Y_position;
long Z_position;
long Zang_position;

 // MicroSwitch pin callouts
const short msLF = 22;   
const short msRF = 23;    
const short msLR = 24;   
const short msRR = 25;  
const short msX = 26;    
const short msY = 27;

const short ZSpeed = 5500;  
const short XSpeed = 7000;
const short YSpeed = 7000;
const short RSpeed = 1000;
const short ZAccel = 2000;
const short XAccel = 1000;
const short YAccel = 1000;
const short RAccel = 500;

const short zSeatSpeed = -130;
const short xSeatSpeed = -130;
const short ySeatSpeed = -130;

const short zCalSpeed = 25;
const short xCalSpeed = 25;
const short yCalSpeed = 25;

const float ZlfInitial = 2.11;
const float ZrfInitial = 0.99;
const float ZlrInitial = 3.59;
const float ZrrInitial = 3.52;
const int xInitial = 0;
const int yInitial = 0;

// Step per linear motion callouts
const float Zmm = 157.4803149; //314.9606299;
const float Xmm = 328.0839896; //1640.419948;
const float Ymm = 328.0839896; //1640.419948;
const float Rmm = 157.48031496; //787.4015748;

//state call-outs
void stateIdle(void);        //idle state is active when node isn't being actively requested 
void stateX(void);           //runs x-axis loop, active when node and x axis is requested by controller
void stateY(void);           //runs y-axis loop, active when node and y axis is requested by controller 
void stateZ(void);           //runs z-axis loop, active when node and z axis is requested by controller
void stateZ_Ang(void);       //runs Z rear axis loop
void stateCalibration();      // Calibration
void stateDiagnostics(void);   //loop which runs through all axis encoders and diagnostic calculations
void stopMotors(void);         //state which stops all motor movement
                       
int X_target = 25;                 
int Y_target = 25;
int Z_target = 25;
int Zang_target;

int state;                                 //state assignments for debug
bool diagCompleted = false;             //flag set by diag state
bool initializationComplete = false;    //flag set by loop
int from_state;
int from_stateOriginal;                            //integer set to track who sent the request for diagnostic or motor stop, in order to send back to the correct loop after
bool faultResolved = false;                     //set by stateDiagnostics when fault detecting fault has been mechanically resolved 
int debug = 0;                              
bool motorsStopped = false;
bool calibrationComplete = true;  

//stepper driver initialization
AccelStepper stepperZlf(1, 2, 70);     //Zlf stepper on driver mode using pin 30 for step and pin 31 for direction
AccelStepper stepperZrf(1, 3, 71);     //Zrf stepper on driver mode using pin 32 for step and pin 33 for direction               
AccelStepper stepperZlr(1, 4, 72);     //Zlr stepper on driver mode using pin 34 for step and pin 35 for direction
AccelStepper stepperZrr(1, 5, 73);     //Zll stepper on driver mode using pin 36 for step and pin 37 for direction
AccelStepper stepperX(1, 6, 76);     //Zlf stepper on driver mode using pin 30 for step and pin 31 for direction
AccelStepper stepperY(1, 7, 77);     //Zrf stepper on driver mode using pin 32 for step and pin 33 for direction               
AccelStepper stepperR(1, 40, 41);


void setup (){
    
  initCan1();
  
  canMod1.attachInterrupt(doCan1Interrupt);
  
  // MicroSwitch pins set to INPUT
  // pinMode(msLF, INPUT);
  // pinMode(msRF, INPUT);
  // pinMode(msLR, INPUT);
  // pinMode(msRR, INPUT);
  // pinMode(msX, INPUT);
  // pinMode(msY, INPUT);

  stepperZlf.setMaxSpeed(ZSpeed);  // Max speed allowable only 
  stepperZrf.setMaxSpeed(ZSpeed);  // valid in moveTo()/run() function call
  stepperZlr.setMaxSpeed(ZSpeed);  // Set in steps/sec
  stepperZrr.setMaxSpeed(ZSpeed);
  stepperX.setMaxSpeed(XSpeed);
  stepperY.setMaxSpeed(YSpeed);
  stepperZlf.setAcceleration(ZAccel);  // Max acceleration only valid 
  stepperZrf.setAcceleration(ZAccel);  // in moveTo()/run() function call
  stepperZlr.setAcceleration(ZAccel);  // I'm not sure exactly what this number equates to
  stepperZrr.setAcceleration(ZAccel);  // It is not (350/25)=new step speed per call..
  stepperX.setAcceleration(XAccel);
  stepperY.setAcceleration(YAccel);

  stepperZlf.setPinsInverted(true, false, false);
  stepperZrf.setPinsInverted(true, false, false);
  stepperZlr.setPinsInverted(true, false, false);
  stepperZrr.setPinsInverted(true, false, false);

    
  }
  
void loop ()                       //node initialization, add initial diagnostic routines here.
{  
  if(diagCompleted == false){     //check if diagnostics have been completed 
    stateDiagnostics();           //run diagnostics loop
  }

  txCAN1(T_SID);
  delay(3);           

  if(T_3 != 0x00){                //check if diagnostic byte has fault present
    txCAN1(T_SID);                   // if fault has been set, transmit fault through CAN to alert controller
    diagCompleted = false;        // reset diagCompleted to false in order to allow diagnostic loop to check if issue has been remedied
  }else {                            
    initializationComplete = true; //otherwise set this flag to true to allow stateIdle to begin 
    stateIdle();                    // proceed to stateIdle. 
  }
}  
                       

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////idle state active while module is not actively being requested- looks for request signal//////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void stateIdle (void)               
{
 
  if (motorsStopped == false){stopMotors();}

  while (initializationComplete == true){      //while this flag is true, continue looping the following code
 
    if (T_3 != 0x00){                           //if diagnostic byte has code proceed with this loop
    txCAN1(T_SID);                              //transmit CAN to notify of fault present
    stateDiagnostics();}                        //run stateDiagnostic to begin monitoring for fault resolution

    if (faultResolved == true)
    {
      txCAN1(T_SID);    // faultResolved variable set by stateDiagnostic, if this is set to 1 transmit on CAN to notify fault has been remedied 
    }     
    faultResolved = false;                          //reset faultResolved to 0

    rxCAN1();
    delay(5);                                   //recieve CAN   

   //Evaluate Node Control Byte and Axis Control Byte 
  if(R_1 == 0x06){                 //check if node control byte is addressed to self         
    txCAN1(T_SID);                 //transmit saved axes positions from last stateDiagnostics routine
    if (R_2 == 0x10){              //if axis control byte is requesting axis 1...
      stateX();}                     // then initiate X axis state
    if (R_2 == 0x08){              
      stateY();}
    if (R_2 == 0x04){
      stateZ();}
    if (R_2 == 0x02){
      stateZ_Ang();}
  }
  if (R_1 == 0x46){
      stateCalibration();}
  }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void stateX (void){
  
  R_1_prev = R_1;                                              //assign values R_1_prev and R_2_prev in order to begin "while" loop 
  R_2_prev = R_2;
  R_4_prev = R_4;

  motorsStopped = false;

  X_position = uint8_t(round(stepperX.currentPosition() / Xmm));
  T_4 = X_position;
  txCAN1(T_SID);
  delay(3);

  while (R_1_prev == 0x06 && R_2_prev == 0x10){                //while R_1_prev and R_2_prev coincide with this node and axis continue running this loop. R_1_prev and R_2_prev are stored values set during this loop.                         
    if(isCAN1MsgReceived == true){
      rxCAN1();                                                            
      delay(3);                                                                                                      //receive CAN
      if (R_SID == 0x414) {                                        //if a new message is received from controller then...                                   
        R_1_prev = R_1;                                             //stores this requested value from controller
        R_2_prev = R_2;                                             //stores this requested value from controller
        R_4_prev = R_4;                                             //stores this requested value from controller
      }
    }
    
    X_target = int(round(R_4_prev * Xmm));
    X_position = uint8_t(round(stepperX.currentPosition() / Xmm));
    
    if(stepperX.targetPosition() != X_target){
      T_2 = 0x30;
      T_4 = X_position;
      txCAN1(T_SID);
      delay(3);    
    }

    stepperX.moveTo(X_target);
    stepperX.run();
    
    if(stepperX.distanceToGo() == 0){
      T_2 = 0x00;
      T_4 = X_position;
      txCAN1(T_SID);
      return;
    }
    
  }

  X_position = uint8_t(round(stepperX.currentPosition() / Xmm));
  T_4 = X_position;
  return;               
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void stateY (void){
  
  R_1_prev = R_1;                                              //assign values R_1_prev and R_2_prev in order to begin "while" loop 
  R_2_prev = R_2;
  R_5_prev = R_5;

  motorsStopped = false;

  Y_position = uint8_t(round(stepperY.currentPosition() / Ymm));
  T_5 = Y_position;
  txCAN1(T_SID);
  delay(3);

  while (R_1_prev == 0x06 && R_2_prev == 0x08){                //while R_1_prev and R_2_prev coincide with this node and axis continue running this loop. R_1_prev and R_2_prev are stored values set during this loop.                         
    if(isCAN1MsgReceived == true){
      rxCAN1();                                                            
      delay(3);                                                                                                      //receive CAN
      if (R_SID == 0x414) {                                        //if a new message is received from controller then...                                   
        R_1_prev = R_1;                                             //stores this requested value from controller
        R_2_prev = R_2;                                             //stores this requested value from controller
        R_5_prev = R_5;                                             //stores this requested value from controller
      }
    }
    
    Y_target = int(round(R_5_prev * Ymm));
    Y_position = uint8_t(round(stepperY.currentPosition() / Ymm));
    
    if(stepperY.targetPosition() != Y_target){
      T_2 = 0x28;
      T_5 = Y_position;
      txCAN1(T_SID);
      delay(3);    
    }

    stepperY.moveTo(Y_target);
    stepperY.run();
    
    if(stepperY.distanceToGo() == 0){
      T_2 = 0x00;
      T_5 = Y_position;
      txCAN1(T_SID);
      return;
    }
    
  }

  Y_position = uint8_t(round(stepperY.currentPosition() / Ymm));
  T_5 = Y_position;
  return;                                   
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void stateZ (void) 
{                                                                      

  R_1_prev = R_1;                                              //assign values R_1_prev and R_2_prev in order to begin "while" loop 
  R_2_prev = R_2;
  R_6_prev = R_6;

  motorsStopped = false;

  Z_position = uint8_t(round(stepperZlf.currentPosition() / Zmm));
  T_6 = Z_position;
  txCAN1(T_SID);
  delay(3);
    
  while(R_1_prev == 0x06 && R_2_prev == 0x04){
    if(isCAN1MsgReceived == true){
      rxCAN1();
      delay(3);
      if(R_SID == 0x414){
        R_1_prev = R_1;
        R_2_prev = R_2;
        R_6_prev = R_6;
      }     
    }

    Z_target = int(round(R_6_prev * Zmm));
    Z_position = uint8_t(round(stepperZlf.currentPosition() / Zmm));

    if(stepperZlf.targetPosition() != Z_target || stepperZrf.targetPosition() != Z_target ||
      stepperZlr.targetPosition() != Z_target || stepperZrr.targetPosition() != Z_target){
      T_2 = 0x24;
      T_6 = Z_position;
      txCAN1(T_SID);
      delay(3);
    }
    stepperZlf.moveTo(Z_target);
    stepperZrf.moveTo(Z_target);
    stepperZlr.moveTo(Z_target);
    stepperZrr.moveTo(Z_target);
    stepperZlf.run();
    stepperZrf.run();
    stepperZlr.run();
    stepperZrr.run();
    
    if(stepperZlf.distanceToGo() == 0 && stepperZrf.distanceToGo() == 0 &&
    stepperZlr.distanceToGo() == 0 && stepperZrr.distanceToGo() == 0){
      T_2 = 0x00;
      T_6 = Z_position;
      txCAN1(T_SID);
      delay(3);
      return;
    }                    
  }
  
  Z_position = uint8_t(round(stepperZlf.currentPosition() / Zmm));
  T_6 = Z_position;
  return;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void stateZ_Ang (void)
{

  R_1_prev = R_1;                                              //assign values R_1_prev and R_2_prev in order to begin "while" loop 
  R_2_prev = R_2;

  motorsStopped = false;

  while (R_1_prev == 0x06 && R_2_prev == 0x02){                //while R_1_prev and R_2_prev coincide with this node and axis continue running this loop. R_1_prev and R_2_prev are stored values set during this loop.
    
  rxCAN1();                                                    //receive CAN


  if (R_SID == 0x414) {                                        //if a new message is received from controller then...                                   
   R_1_prev = R_1;                                             //stores this requested value from controller
   R_2_prev = R_2;                                             //stores this requested value from controller
   R_7_prev = R_7;                                             //stores this requested value from controller
  }
     
  
 //  if (Zrear_target != Zrear_position){                                                                            //if axis target (requested coordinate from controller) does not equal axis position (current posiiton) then begin loop
 //   if(Zrear_position < Zrear_target){digitalWrite (zrear_led_up, HIGH); (zrear_led_down, LOW); T_2 = 0x22;}            //if axis position is less then axis target then illuminate LED "forward" and set T_2 to 0x30 to indicate adjustment in progress
 //   else if(Zrear_position > Zrear_target) {digitalWrite (zrear_led_down, HIGH); (zrear_led_up, LOW); T_2 = 0x22;}}     //if axis position is less then axis target then illuminate LED "back" and set T_2 to 0x30 to indicate adjustment in progress                                      
 //   else {digitalWrite (zrear_led_up, LOW); digitalWrite (zrear_led_down, LOW); T_2 = 0x00;}                     //if target and position do match, then turn off LEDs and set T_2 to 0x00 to indicate adjustment done  
 
  txCAN1(T_SID);                                                 //transmit CAN
  }
    
    stateIdle();                                               //R_1 and R_2 fail to match this node or axis at "while" condition then return to stateIdle loop 

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
void stateDiagnostics (void) 
{
  
  if (motorsStopped == false) {stopMotors();} 
  
  while(calibrationComplete == false){
    T_3 = 0x21;
    txCAN1(T_SID);
    delay(3);
    stateCalibration();
  }

  if(calibrationComplete == true){
    T_3 = 0x00;
    faultResolved = true;
    diagCompleted = true;
  }  
   
  loop();
            
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void stopMotors(void)
{
  stepperZlf.stop();
  stepperZrf.stop();
  stepperZlr.stop();
  stepperZrr.stop();
  stepperX.stop();
  stepperY.stop();

  motorsStopped = true;

  return;
}

void stateCalibration(void)
{
  if(isCAN1MsgReceived == true){
    rxCAN1();
  }

  //  MicroSwitch and Calibration conditions 
  bool Zcal = true;    // These are conditional states that need to 
  bool Zhome = true;   // change in calibration, but also they need to 
  bool Xcal = true;    // set back to their original states, that's why they 
  bool Xhome = true;   // are declared in this scope.
  bool Ycal = true;
  bool Yhome = true;

  if(calibrationComplete == false){
    Z_target = int(round(Z_target * Zmm));
    X_target = int(round(X_target * Xmm));
    Y_target = int(round(Y_target * Ymm));
  }


  if(R_SID == 0x414 && R_1 == 0x46){
    T_2 = 0x40;
    txCAN1(T_SID);
    delay(5);
  
  // while(Zhome == false){    //Checks for initial seating...
  
  //   while(digitalRead(msLF) == HIGH || digitalRead(msRF) == HIGH    // Remain in this loop as long
  //   || digitalRead(msLR) == HIGH || digitalRead(msRR) == HIGH){   // as any motor is not seated 
  //     if(digitalRead(msLF) == HIGH){  // Run each motor independantly to fast seat position and wait
  //       stepperZlf.setSpeed(zSeatSpeed);  // for all others to loop until seated.
  //       stepperZlf.runSpeed();
  //     }
  //     if(digitalRead(msRF) == HIGH){  
  //       stepperZrf.setSpeed(zSeatSpeed);
  //       stepperZrf.runSpeed();
  //     }
  //     if(digitalRead(msLR) == HIGH){
  //       stepperZlr.setSpeed(zSeatSpeed);
  //       stepperZlr.runSpeed();
  //     }
  //     if(digitalRead(msRR) == HIGH){
  //       stepperZrr.setSpeed(zSeatSpeed);
  //       stepperZrr.runSpeed();
  //     }
  //     if(digitalRead(msLF) == LOW && digitalRead(msRF) == LOW &&  // When all motors are seated Zhome condition
  //       digitalRead(msLR) == LOW && digitalRead(msRR) == LOW){  // change kicks us out of loop.
  //       Zhome = true;
  //     }
  //   }
  // }
  // delay(5);
  // while(Zcal == false && Zhome == true){    // Checks if final Calibration is complete..
    
  //   while(digitalRead(msLF) == LOW || digitalRead(msRF) == LOW    // Remain in this loop as long as any..
  //   || digitalRead(msLR) == LOW || digitalRead(msRR) == LOW){ // motor is still seated.
  //     if(digitalRead(msLF) == LOW){ // Run each motor independantly slowly out of Microswitch position.
  //       stepperZlf.setSpeed(zCalSpeed);
  //       stepperZlf.runSpeed();
  //     } else {
  //       stepperZlf.stop();         // Once out of MicroSwitch range stop motor,
  //       stepperZlf.setCurrentPosition(int(round(ZlfInitial * Zmm)));  // set open loop stepper position (based on step count),
  //     }                   // and wait for all others to follow.
  //     if(digitalRead(msRF) == LOW){
  //       stepperZrf.setSpeed(zCalSpeed);
  //       stepperZrf.runSpeed();
  //     } else {
  //       stepperZrf.stop();
  //       stepperZrf.setCurrentPosition(int(round(ZrfInitial * Zmm)));
  //     }
  //     if(digitalRead(msLR) == LOW){
  //       stepperZlr.setSpeed(zCalSpeed);
  //       stepperZlr.runSpeed();
  //     } else {
  //       stepperZlr.stop();
  //       stepperZlr.setCurrentPosition(int(round(ZlrInitial * Zmm)));
  //     }
  //     if(digitalRead(msRR) == LOW){
  //       stepperZrr.setSpeed(zCalSpeed);
  //       stepperZrr.runSpeed();
  //     } else {
  //       stepperZrr.stop();
  //       stepperZrr.setCurrentPosition(int(round(ZrrInitial * Zmm)));
  //     }
  //     if(digitalRead(msLF) == HIGH && digitalRead(msRF) == HIGH   // When all motors have completed slow rise
  //     && digitalRead(msLR) == HIGH && digitalRead(msRR) == HIGH){ // (which learns current position in the loop)
  //       Zcal = true;                        // kick us out of calibration loop.
  //       stepperZlf.moveTo(Z_target);
  //       stepperZrf.moveTo(Z_target);
  //       stepperZlr.moveTo(Z_target);
  //       stepperZrr.moveTo(Z_target);
  //     }
  //   }
  // }
  // delay(5);
  // while(Xhome == false){   

  //   if(digitalRead(msX) == HIGH){
  //     stepperX.setSpeed(xSeatSpeed);
  //     stepperX.runSpeed();
  //   } else {
  //     Xhome = true;
  // }
  // delay(5);
  // while(Xcal == false && Xhome == true){
    
  //   if(digitalRead(msX) == LOW){
  //     stepperX.setSpeed(xCalSpeed);
  //     stepperX.runSpeed();
  //   } else {
  //     stepperX.stop();
  //     stepperX.setCurrentPosition(xInitial);
  //   }
  //   if(msX == HIGH){
  //     Xcal = true;
  //     stepperX.moveTo(X_target);
  //   }
  // }
  // delay(5);  
  // while(Yhome == false){
      
  //   if(digitalRead(msY) == HIGH){
  //     stepperY.setSpeed(ySeatSpeed);
  //     stepperY.runSpeed();
  //   } else {
  //     Yhome = true;
  //   }
  // }
  // delay(5);
  // while(Ycal == false && Yhome == true){
    
  //   if(digitalRead(msY) == LOW){
  //     stepperY.setSpeed(yCalSpeed);
  //     stepperY.runSpeed();
  //   } else {
  //     stepperY.stop();
  //     stepperY.setCurrentPosition(yInitial);
  //   }
  //   if(msY == HIGH){
  //     Ycal = true;
  //     stepperY.moveTo(Y_target);
  //   }
  // } 
  delay(5);
  if(Zcal == true && Xcal == true && Ycal == true){
    // while(stepperZlf.distanceToGo() != 0 || stepperZrf.distanceToGo() != 0 ||
    //   stepperZlr.distanceToGo() != 0 || stepperZrr.distanceToGo() != 0 ||
    //   stepperX.distanceToGo() != 0 || stepperY.distanceToGo() != 0){
    //     stepperZlf.run();
    //     stepperZrf.run();
    //     stepperZlr.run();
    //     stepperZrr.run();
    //     stepperX.run();
    //     stepperY.run();
    // }
      delay(5);
      calibrationComplete = true;
      T_2 = 0x00;
      T_4 = uint8_t(round(stepperX.currentPosition() / Xmm));
      T_5 = uint8_t(round(stepperY.currentPosition() / Ymm));
      T_6 = uint8_t(round(stepperZlf.currentPosition() / Zmm));
      txCAN1(T_SID);
      delay(3);
  }
  return;
  }
  // }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////CAN LOOPS/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////

void initCan1(void) 
  {
  CAN::BIT_CONFIG canBitConfig;
  canMod1.enableModule(true);
  canMod1.setOperatingMode(CAN::CONFIGURATION);
  while(canMod1.getOperatingMode() != CAN::CONFIGURATION);			
  canBitConfig.phaseSeg2Tq            = CAN::BIT_3TQ;
  canBitConfig.phaseSeg1Tq            = CAN::BIT_3TQ;
  canBitConfig.propagationSegTq       = CAN::BIT_3TQ;
  canBitConfig.phaseSeg2TimeSelect    = CAN::TRUE;
  canBitConfig.sample3Time            = CAN::TRUE;
  canBitConfig.syncJumpWidth          = CAN::BIT_2TQ;
  canMod1.setSpeed(&canBitConfig,SYS_FREQ,CAN_BUS_SPEED);
  canMod1.assignMemoryBuffer(CAN1MessageFifoArea,2 * 8 * 16);	
  canMod1.configureChannelForTx(CAN::CHANNEL0,8,CAN::TX_RTR_DISABLED,CAN::LOW_MEDIUM_PRIORITY);
  canMod1.configureChannelForRx(CAN::CHANNEL1,8,CAN::RX_FULL_RECEIVE);
  canMod1.configureFilter      (CAN::FILTER0, R_Filter, CAN::SID);    
  canMod1.configureFilterMask  (CAN::FILTER_MASK0, R_Mask, CAN::SID, CAN::FILTER_MASK_IDE_TYPE);
  canMod1.linkFilterToChannel  (CAN::FILTER0, CAN::FILTER_MASK0, CAN::CHANNEL1); 
  canMod1.enableFilter         (CAN::FILTER0, true);
  canMod1.enableChannelEvent(CAN::CHANNEL1, CAN::RX_CHANNEL_NOT_EMPTY, true);
  canMod1.enableModuleEvent(CAN::RX_EVENT, true);
  canMod1.setOperatingMode(CAN::NORMAL_OPERATION);
  while(canMod1.getOperatingMode() != CAN::NORMAL_OPERATION);			
  }

void txCAN1(uint32_t tx)
  {  
  CAN::TxMessageBuffer * message;
  message = canMod1.getTxMessageBuffer(CAN::CHANNEL0);
  if (message != NULL)
    {
    message->messageWord[0] = 0;
    message->messageWord[1] = 0;
    message->messageWord[2] = 0;
    message->messageWord[3] = 0;
    message->msgSID.SID   = tx;		
    message->msgEID.IDE   = 0;			
    message->msgEID.DLC   = 8;
    }    			
  message->data[0]      = T_1;
  message->data[1]      = T_2;
  message->data[2]      = T_3;
  message->data[3]      = T_4;
  message->data[4]      = T_5;
  message->data[5]      = T_6;
  message->data[6]      = T_7;
  message->data[7]      = T_8;
  canMod1.updateChannel(CAN::CHANNEL0);
  canMod1.flushTxChannel(CAN::CHANNEL0);
  }	

void rxCAN1(void)
  {
  CAN::RxMessageBuffer * message;
  if (isCAN1MsgReceived == false) //if no messages in buffer set all variables to 0 and go back to main loop
    {
    R_SID = 0; 
    R_1 = 0;
    R_2 = 0;
    R_3 = 0;
    R_4 = 0;
    R_5 = 0;
    R_6 = 0;
    R_7 = 0;
    R_8 = 0;
    return;
    }                            //message available so clear received flag and save message SID and data to variables
  isCAN1MsgReceived = false;	
  message = canMod1.getRxMessage(CAN::CHANNEL1);
  R_SID = message->msgSID.SID;
  R_1 = message->data[0];
  R_2 = message->data[1];
  R_3 = message->data[2];
  R_4 = message->data[3];
  R_5 = message->data[4];
  R_6 = message->data[5];
  R_7 = message->data[6];
  R_8 = message->data[7];
  canMod1.updateChannel(CAN::CHANNEL1);
  canMod1.enableChannelEvent(CAN::CHANNEL1, CAN::RX_CHANNEL_NOT_EMPTY, true);
  }

void doCan1Interrupt()
  {
  if ((canMod1.getModuleEvent() & CAN::RX_EVENT) != 0)
    {
    if(canMod1.getPendingEventCode() == CAN::CHANNEL1_EVENT)
      {
      canMod1.enableChannelEvent(CAN::CHANNEL1, CAN::RX_CHANNEL_NOT_EMPTY, false);
      isCAN1MsgReceived = true;	
      }
    }
  }
