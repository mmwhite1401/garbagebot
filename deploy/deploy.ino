// Libraries
#include <ICM_20948.h> // Accellerometer Library (SparkFun 9DoF IMU Breakout)

// Definitions
ICM_20948_I2C Accel1;  // Creates Accelerometer Object in I2C
#define AD0_VAL 0 // Value of the last bit of the I2C address
#define WIRE_PORT Wire
#define SERIAL_PORT Serial

// Variables
// Pins
int AN2 = 5;       //PIN -- PWM output (right)
int AN1 = 6;       //PIN -- PWM output (left)
int IN2 = 7;       //PIN -- Motor direction output (right)
int IN1 = 8;       //PIN -- Motor direction output (left)
int RC_R = 10;     //PIN -- RC input   (right), Channel 2
int RC_L = 3;      //PIN -- RC input   (left), Channel 3
int AUTOLGT = 4;  // PIN -- Automode
int manualscoopswitch = 50; // Pin for manual scoop switch input, Switch F, Channel 5
int pilotbutton = 52; // Pin to give the greenlight for autopath, Switch B, Channel 9
int emergencystop = 2; // Emergency shutoff pin

int ultraLtrig = 22, ultraLecho = 23; // left ultrasound pins
int ultraRtrig = 24, ultraRecho = 25; // right ultrasound pins

int rightbicep = 30; // low to move, high to stop, motor 1
int leftbicep = 31; // motor 2
int leftforearm = 32; // motor 3
int rightforearm = 33; // motor 4
int armdirection = 34; // low retracts, high extends
unsigned long armtime = 0;

// Drive Motor PWMs
int PWM_R = 1500;         //SIGNAL RIGHT
int PWM_L = 1500;         //SIGNAL LEFT
int PWMMIN = 0, PWMMAX = 150; // min and max PWM values
int pwrstep = 1; // how large a change of pwr per step

// Compass Variables
float compema = 0.15; // ema for the compass
float compass = 0; // current compass
float avgcomp = 0; // averagecompass value

// Pathing Variables
int pilotsignal = 2000; // variable for pilotsignal transmitter
bool panicking = false; // are we emergency stopped?
unsigned long drivetime = 0; // Current drivetime
unsigned long pauseddrivetime = 0; // time stopped for obstacles
unsigned long maxdrivetime = 10000; // Maximum drivetime (milliseconds)
int turns = 0, maxturns = 10; // how many columns to clear
int LR = 1; // will bot turn left or right next? 0 = left, 1 = right
float pathcompass = 0; // path compass value
int compassconstraint = 2; // min and max deviation of compass for autodriveForward
bool driveline = false; // are we driving forward?
bool turning = false; // are we turning?
bool turnComplete = false; // is the turn complete?
bool calibrated = false; // are we calibrated?

// Ultrasound Variables
int ultraDistanceL = 0; // left ultrasound, raw distance
int ultraAvgL = 0; // left ultrasound, averaged for noise
int ultraDistanceR = 0; // right ultrasound, raw distance
int ultraAvgR = 0; // right ultrasound, averaged for noise
long duration = 0; // used for calculating distance
int ultraObstacle = 300; // check for obstacles within this distance
double ultraema = 0.20; // Use exponential moving average.
bool objectdetected = false;

// Scoop Variables
bool scoopmode = false; // True for automatic mode
bool scoopdigging = false;
bool scoopraised = false;
float scoopheight = 0;
int manualscoopsignal = 0; // Manualscoop at >1900
int scoopstate = 0; //sets state for manual control
int scoopstatemem = 0; // rememebers last state for manual control





void setup() {
  // put your setup code here, to run once:
  analogWrite(AN1, 0);    //SETUP DRIVE INPUTS TO 0
  analogWrite(AN2, 0);    //SETUP DRIVE INPUTS TO 0
  digitalWrite(AUTOLGT, LOW); //warning, steerclear, setting up
  Serial.println("Light ON");
  analogWrite(IN1, -1);    //SETUP FWD DRIVE
  analogWrite(IN2, -1);    //SETUP FWD DRIVE


  panicking = false;

  pinMode(emergencystop, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(emergencystop), panic, RISING); // emergency stop interrupt
  pinMode(RC_R, INPUT);
  pinMode(RC_L, INPUT);
  pinMode(ultraLecho, INPUT);
  pinMode(ultraRecho, INPUT);
  pinMode(pilotbutton, INPUT);
  pinMode(manualscoopswitch, INPUT);

  pinMode(IN2, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(ultraLtrig, OUTPUT);// left ultrasound pins
  pinMode(ultraRtrig, OUTPUT);// right ultrasound pins
  pinMode(leftbicep, OUTPUT);
  pinMode(rightbicep, OUTPUT);
  pinMode(leftforearm, OUTPUT);
  pinMode(rightforearm, OUTPUT);
  pinMode(armdirection, OUTPUT);
  pinMode(AUTOLGT, OUTPUT);

  digitalWrite(leftbicep, HIGH);
  digitalWrite(rightbicep, HIGH);
  digitalWrite(leftforearm, HIGH);
  digitalWrite(rightforearm, HIGH);
  digitalWrite(armdirection, HIGH);
  scoopstatemem = 0;

  // Setup Serial and I2C
  SERIAL_PORT.begin(115200);
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);

  // Initialize the IMU
  bool initialized = false;
  while (!initialized)
  {
    Accel1.begin(WIRE_PORT, AD0_VAL);

    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(Accel1.statusString());
    if (Accel1.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }
  bool success = true; // Use success to show if the DMP configuration was successful

  success &= (Accel1.initializeDMP() == ICM_20948_Stat_Ok);
  // Enable DMP Sensors
  success &= (Accel1.enableDMPSensor(INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD) == ICM_20948_Stat_Ok);

  // Set the data rate of the geomagnetic compass
  int fusionrate = 4;
  success &= (Accel1.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, fusionrate) == ICM_20948_Stat_Ok); // Set to 225Hz

  // Enable the FIFO
  success &= (Accel1.enableFIFO() == ICM_20948_Stat_Ok);

  // Enable the DMP
  success &= (Accel1.enableDMP() == ICM_20948_Stat_Ok);

  // Reset DMP
  success &= (Accel1.resetDMP() == ICM_20948_Stat_Ok);

  // Reset FIFO
  success &= (Accel1.resetFIFO() == ICM_20948_Stat_Ok);

  // Check success
  if (success)
  {
    SERIAL_PORT.println(F("DMP enabled!"));
  }
  else
  {
    SERIAL_PORT.println(F("Enable DMP failed!"));
    SERIAL_PORT.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
    while (1)
      ; // Do nothing more
  }
  digitalWrite(AUTOLGT, HIGH); //warning, steerclear, setting up
  Serial.println("Light OFF");
}






void loop() {
  if(digitalRead(emergencystop)== HIGH || panicking == true)
  {
    panicking = true;

    
      analogWrite(AN1, 0);
      analogWrite(AN2, 0);
      
   
    digitalWrite(AUTOLGT, LOW);
    Serial.println("Light ON");
    
    if(scoopstatemem > 0)
    {
    digitalWrite(leftbicep, HIGH);
    digitalWrite(rightbicep, HIGH);
    digitalWrite(leftforearm, HIGH);
    digitalWrite(rightforearm, HIGH);
    digitalWrite(armdirection, HIGH);
    Serial.println("Panic set relays high");
    scoopstatemem = 0; 
    }
    
    Serial.print("Panic!!!\n");
    delay(1000);
    return;
  }else{
   PWM_L = pulseIn(RC_L, HIGH); //Read left stick inputs
   PWM_R = pulseIn(RC_R, HIGH); //Read right stick inputs
   
   manualscoopsignal = pulseIn(manualscoopswitch, HIGH); //read scoop mode switch
   Serial.print("manualscoopsignal in");
   if (manualscoopsignal > 1500)
   {
    Serial.print(" Scoopmode\n");
   }else{
    Serial.print(" Drive mode\n");
   }
   
   pilotsignal = pulseIn(pilotbutton, HIGH); //read drive mode switch
   Serial.print("pilotsignal in");
   if (pilotsignal > 1500)
   {
    Serial.print("Drive Manual\n");
   }else{
    Serial.print("Drive Autonomous\n");
   }
  }
  
  if(manualscoopsignal > 1900 && panicking == false)
  {
    Serial.print("Updatescoop\n");
      analogWrite(AN1, 0);
      analogWrite(AN2, 0);
    manualscoop();
    return;
  }

  if(pilotsignal > 1600 && panicking == false)
  {
    
    if(scoopstatemem != 0)
    {
    scoopstatemem = 0;
    scpcontrol();
    }
    Serial.print("manualdrive\n");
    manualdrive();
    return;
  }
  
  if(pilotsignal < 1600){
    scoopstatemem = 5;  //automated scoop action, but no discernable action.
    if (digitalRead(AUTOLGT) == HIGH)
    {
      digitalWrite(AUTOLGT, LOW);
      Serial.println("Light ON");
    }
    Serial.print("autodrive\n");
    autodrive();
    return;
  }  else{
    panicking = true;
  }
}

void manualscoop() {
  /*
     only one state can be active at a time (0 is the most likely)
     state 0 : no motors moving
     state 1 : biceps FWD
     state 2 : biceps REV
     state 3 : forearm FWD
     state 4 : forearm REV
     state 5 : was in automated mode
  */
  if ((PWM_L >= 1430 && PWM_L <= 1540) && (PWM_R >= 1430 && PWM_R <= 1540 )) // Left stick centered AND Right stick centered OR left stick null OR right stick null
  {
    scoopstate = 0;
  } else if (PWM_L < 1430 && PWM_L > 1000) { // Left stick forward, Biceps FWD
    scoopstate = 1;
  } else if (PWM_L > 1540) {                // Left stick reverse, Biceps REV
    scoopstate = 2;
  } else if (PWM_R < 1430 && PWM_R > 1000) { // Right stick forward, FOREARM FWD
    scoopstate = 3;
  } else if (PWM_R > 1540) {                // Right stick reverse, FOREARM REV
    scoopstate = 4;
  } else {
    panicking = true;
  }

  if (scoopstate == scoopstatemem)
  {
    //Serial.println("state = memory");
    Serial.println(scoopstatemem);
    return;
  } else {
    scoopstatemem = scoopstate;
    scpcontrol();
  }
  return;
}


void manualdrive() {
  // Manual Drive Control

  // LEFT joystick control //
  if ((PWM_L >= 1430 && PWM_L <= 1540)) // LEFT joystick is centered (neither fwd/reverse)
  {
    analogWrite(AN1, 0);
    Serial.println("Left no Drive");
  }
  else if (PWM_L > 1540) //LEFT joystick is in reverse, map PWM values, send signal
  {
    analogWrite(IN1, 1);
    analogWrite(AN1, map(PWM_L, 1539, 1915, 55, 255));
    Serial.println("Left Drive Reverse : ");
    Serial.print(AN1); Serial.print("\n");
  }
  else if (PWM_L < 1430 && PWM_L > 1000) //LEFT joystick is forward, map PWM values, send signal
  {
    analogWrite(IN1, 0);
    analogWrite(AN1, map(PWM_L, 1431, 1080, 55, 255));
    Serial.println("Left Drive Forward : ");
    Serial.print(AN1); Serial.print("\n");
  } else {
    panicking = true;
    Serial.println("panicL");
  }

  // RIGHT joystick control //
  if ((PWM_R >= 1430 && PWM_R <= 1540)) // RIGHT joystick is centered (neither fwd/reverse)
  {
    analogWrite(AN2, 0);
    Serial.println("Right No Drive");
  }
  else if (PWM_R > 1540) //RIGHT joystick is in reverse, map PWM values, send signal
  {
    analogWrite(IN2, 1);
    analogWrite(AN2, map(PWM_R, 1539, 1915, 55, 255));
    Serial.println("Right Drive Reverse : ");
    Serial.print(AN2); Serial.print("\n");
  }
  else if (PWM_R < 1430 && PWM_R > 1000) //RIGHT joystick is forward,  map PWM values, send signal
  {
    analogWrite(IN2, 0);
    analogWrite(AN2, map(PWM_R, 1431, 1080, 55, 255));
    Serial.println("Right Drive Forward : ");
    Serial.print(AN2); Serial.print("\n");
  } else {
    panicking = true;
    Serial.println("panicR");
  }
  return;
}

void autodrive() {
  if (calibrated == false)
  {
    calibrate();
  }
  calcIMU();
  calcPos();
  ultraSound();
  autoPath();
}

void scpcontrol() {
  switch (scoopstatemem) {
    case 0:
      digitalWrite(leftbicep, HIGH);
      digitalWrite(rightbicep, HIGH);
      digitalWrite(leftforearm, HIGH);
      digitalWrite(rightforearm, HIGH);
      digitalWrite(armdirection, HIGH);
      Serial.println("scoop will do nothing");
      break;
    case 1:
      digitalWrite(leftbicep, LOW);
      digitalWrite(rightbicep, LOW);
      digitalWrite(leftforearm, HIGH);
      digitalWrite(rightforearm, HIGH);
      digitalWrite(armdirection, HIGH);
      Serial.println("BICEPS FWD");
      break;
    case 2:
      digitalWrite(leftbicep, LOW);
      digitalWrite(rightbicep, LOW);
      digitalWrite(leftforearm, HIGH);
      digitalWrite(rightforearm, HIGH);
      digitalWrite(armdirection, LOW);
      Serial.println("BICEPS REV");

      break;
    case 3:
      digitalWrite(leftbicep, HIGH);
      digitalWrite(rightbicep, HIGH);
      digitalWrite(leftforearm, LOW);
      digitalWrite(rightforearm, LOW);
      digitalWrite(armdirection, HIGH);
      Serial.println("FOREARM FWD");
      break;
    case 4:
      digitalWrite(leftbicep, HIGH);
      digitalWrite(rightbicep, HIGH);
      digitalWrite(leftforearm, LOW);
      digitalWrite(rightforearm, LOW);
      digitalWrite(armdirection, LOW);
      Serial.println("FOREARM REV");
      break;
    case 5:
      digitalWrite(leftbicep, LOW);
      digitalWrite(rightbicep, LOW);
      digitalWrite(leftforearm, LOW);
      digitalWrite(rightforearm, LOW);
      digitalWrite(armdirection, HIGH);
      Serial.println("BOTH FWD");
      break;
    case 6:
      digitalWrite(leftbicep, LOW);
      digitalWrite(rightbicep, LOW);
      digitalWrite(leftforearm, LOW);
      digitalWrite(rightforearm, LOW);
      digitalWrite(armdirection, LOW);
      Serial.println("BOTH REV");
      break;
    default:
      digitalWrite(leftbicep, HIGH);
      digitalWrite(rightbicep, HIGH);
      digitalWrite(leftforearm, HIGH);
      digitalWrite(rightforearm, HIGH);
      digitalWrite(armdirection, HIGH);
      Serial.println("scoop will do nothing");
      break;
  }
  return;
}

// Emergency Stop ISR. Returns to manual control
void panic()
{
  allStop();
  panicking = true;
}

// IMU Funtions
void calcIMU()
{
  int moreData = readIMUDMP(); // Read data from accelerometer

  if (moreData >= 0) //function returns -1 if no data was read.
  {
    // Average compass value
    avgcomp = avgcomp * (1 - compema) + compass * compema;
  }
}

int readIMUDMP() {
  // Get all IMU Data
  // Read any DMP data waiting in the FIFO
  // Note:
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFONoDataAvail if no data is available.
  //    If data is available, readDMPdataFromFIFO will attempt to read _one_ frame of DMP data.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOIncompleteData if a frame was present but was incomplete
  //    readDMPdataFromFIFO will return ICM_20948_Stat_Ok if a valid frame was read.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOMoreDataAvail if a valid frame was read _and_ the FIFO contains more (unread) data.
  icm_20948_DMP_data_t data;
  Accel1.readDMPdataFromFIFO(&data);

  if ((Accel1.status == ICM_20948_Stat_Ok) || (Accel1.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
  {
    //SERIAL_PORT.print(F("Received data! Header: 0x")); // Print the header in HEX so we can see what data is arriving in the FIFO
    //if ( data.header < 0x1000) SERIAL_PORT.print( "0" ); // Pad the zeros
    //if ( data.header < 0x100) SERIAL_PORT.print( "0" );
    //if ( data.header < 0x10) SERIAL_PORT.print( "0" );
    //SERIAL_PORT.println( data.header, HEX );

    // Update Compass
    if ((data.header & DMP_header_bitmap_Compass_Calibr) > 0) // Check for Compass
    {
      compass = (float)data.Compass_Calibr.Data.Y / 10000; // Extract the compass data
    }
  }
  if (Accel1.status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
  {
    return 0; // No more data
  }
  else
  {
    return 1; // More data
  }
}

void calcPos()
{
  // Calculate drivetime
  if (driveline == false && turning == false)
  {
    digScoop();
    Serial.println("Calculating time at start of drive");
    drivetime = millis();
    maxdrivetime = 10000;
    driveline = true;
  }

  // Calculated paused drivetime
  if (objectdetected == false)
  {
    pauseddrivetime = millis();
  }
  else
  {
    Serial.println("Object detected, pausing drivetime");
    maxdrivetime += millis() - pauseddrivetime;
  }

  // If over drivetime, set driving to false
  if ((millis() - drivetime) > maxdrivetime)
  {
    Serial.println("Over maxdrivetime");
    driveline = false;
  }
}

// calibrate compass and set path direction
void calibrate()
{
  Serial.println("Calibrating compass");
  // Calibrate compass
  for (int index = 0; index <= 100; index++)
  {
    int moreData = readIMUDMP();
    if (moreData >= 0) //function returns -1 if no data was read.
    {
      avgcomp = avgcomp * (1 - compema) + compass * compema;
    }
  }

  Serial.println("Setting path direction");
  // Set path direction
  pathcompass = avgcomp;
  calibrated = true;
}

// Pathing Functions
void autoPath()
{
  if (panicking == true);
  {
    return;
  }
  // Path code
  if (objectdetected == true)
  {
    Serial.println("Object detected, stopping");
    allStop();
    return;
  }

  // Keep going until max turns reached
  if (turns < maxturns)
  {
    // while within time bound, drive forward
    if (driveline == true)
    {
      autodriveForward(); // drive forward
      turning = false;
    }
    else
    {
      if (turning == false)
      {
        Serial.println("Stopping from autodrive forward");
        allStop(); // stop driving
        depositScoop();
        turning = true;
      }
    }

    if (turnComplete == false && turning == true)
    {
      // Turn left or right
      switch (LR)
      {
        case 0:
          autoturnLeft(); // turn left
          break;

        case 1:
          autoturnRight();  // turn right
          break;
      }
    }
  }
  else
  {
    allStop();
  }
}


// Forward drive path
void autodriveForward()
{
  if (panicking == true);
  {
    return;
  }
  Serial.println("Auto driving forward");
  // If speed under max, increase power, constrain to PWM
  if (PWM_L < PWMMAX || PWM_R < PWMMAX)
  {
    PWM_L += pwrstep;
    PWM_R += pwrstep;

    PWM_L = constrain(PWM_L, PWMMIN, PWMMAX);
    PWM_R = constrain(PWM_R, PWMMIN, PWMMAX);
  }

  // If drifting right, correct left
  if (abs(pathcompass - avgcomp) > compassconstraint)
  {
    PWM_L -= 2 * pwrstep;
    PWM_R += 2 * pwrstep;

    PWM_L = constrain(PWM_L, PWMMIN, PWMMAX);
    PWM_R = constrain(PWM_R, PWMMIN, PWMMAX);
  }

  // If drifting left, correct right
  if (abs(avgcomp - pathcompass) > compassconstraint)
  {
    PWM_L += 2 * pwrstep;
    PWM_R -= 2 * pwrstep;

    PWM_L = constrain(PWM_L, PWMMIN, PWMMAX);
    PWM_R = constrain(PWM_R, PWMMIN, PWMMAX);
  }

  // Write values to motors
  analogWrite(IN1, -1);
  analogWrite(AN1, PWM_L);
  analogWrite(IN2, -1);
  analogWrite(AN2, PWM_R);
}

void allStop()
{
  Serial.println("Stopping");
  // Slowly set PWMs to 0
  while (PWM_L > 0 || PWM_R > 0)
  {
    PWM_L -= pwrstep;
    PWM_R -= pwrstep;

    PWM_L = constrain(PWM_L, 0, PWMMAX);
    PWM_R = constrain(PWM_R, 0, PWMMAX);

    // Write values to motors
    analogWrite(IN1, -1);
    analogWrite(AN1, PWM_L);
    analogWrite(IN2, -1);
    analogWrite(AN2, PWM_R);

    delay(10);
  }
}

void autoturnLeft()
{
  if (panicking == true);
  {
    return;
  }
  Serial.println("Auto turning left");
  if ((avgcomp / pathcompass) > -0.99 || (avgcomp / pathcompass) < -1.01)
  {

    PWM_R += pwrstep;
    PWM_R = constrain(PWM_R, PWMMIN, PWMMAX);

    analogWrite(IN2, -1);
    analogWrite(AN2, PWM_R);
  }
  else
  {
    turnComplete = true;
  }
  if (turnComplete == true)
  {
    Serial.println("Auto Left turn complete");
    allStop();
    driveline = false;
    turning = false;
    turnComplete = false;
    LR = 1;
    turns += 1;
    pathcompass = -pathcompass;
  }
}

void autoturnRight()
{
  if (panicking == true);
  {
    return;
  }
  Serial.println("Auto turning right");
  if ((avgcomp / pathcompass) > -0.99 || (avgcomp / pathcompass) < -1.01)
  {

    PWM_L += pwrstep;
    PWM_L = constrain(PWM_L, PWMMIN, PWMMAX);

    analogWrite(IN1, -1);
    analogWrite(AN1, PWM_L);
  }
  else
  {
    turnComplete = true;
  }
  if (turnComplete == true)
  {
    Serial.println("Auto right turn complete");
    allStop();
    driveline = false;
    turning = false;
    turnComplete = false;
    LR = 0;
    turns += 1;
    pathcompass = -pathcompass;
  }
}

// Sensor functions
void ultraSound()
{
  // If scoop deployed, driving, and object within 3 feet, disable greenlight
  // Ultrasound Recording
  // Trigger the left ultrasound pulse
  Serial.println("Checking left ultrasound");
  digitalWrite(ultraLtrig, LOW);
  delayMicroseconds(2);
  digitalWrite(ultraLtrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(ultraLtrig, LOW);
  // Capture the return
  duration = pulseIn(ultraLecho, HIGH);
  if (duration * 0.034 / 2 < 500)
  {
    ultraDistanceL = duration * 0.034 / 2;
  }
  else
  {
    ultraDistanceL = 500;
  }

  // Trigger the right ultrasound pulse
  Serial.println("Checking right ultrasound");
  digitalWrite(ultraRtrig, LOW);
  delayMicroseconds(2);
  digitalWrite(ultraRtrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(ultraRtrig, LOW);
  // Capture the return
  duration = pulseIn(ultraRecho, HIGH);
  if (duration * 0.034 / 2 < 500)
  {
    ultraDistanceR = duration * 0.034 / 2;
  }
  else
  {
    ultraDistanceR = 500;
  }

  // Calculate Ultrasound avgs
  ultraAvgL = ultraAvgL * (1 - ultraema) + ultraDistanceL * ultraema;
  ultraAvgR = ultraAvgR * (1 - ultraema) + ultraDistanceR * ultraema;

  // Detect if an object is within the specified distance and stop
  if (ultraAvgL < ultraObstacle || ultraAvgR < ultraObstacle)
  {
    objectdetected = true;
  }
  else
  {
    objectdetected = false;
  }
}

// Scoop Functions
// start digging
void digScoop()
{
  Serial.println("digScoop called");
  // if already digging, return
  if (scoopdigging == true && scoopraised == false)
  {
    Serial.println("Already digging");
    return;
  }

  // if raised, reset then dig
  if (scoopdigging == false && scoopraised == true)
  {
    Serial.println("Raised, resetting before dig");
    resetScoop();
  }

  Serial.println("Lowering scoop to dig");
  // placeholder for scoop deploy code
  delay(1);

  scoopdigging = true;
}

// deposit into bin
void depositScoop()
{
  Serial.println("depositScoop called");
  // if already depositing, return
  if (scoopdigging == false && scoopraised == true)
  {
    Serial.println("Already depositing");
    return;
  }

  // if currently digging, reset scoop first
  if (scoopdigging == true && scoopraised == false)
  {
    Serial.println("Digging, resetting before depositing");
    resetScoop();
  }

  Serial.println("Depositing scoop");
  armtime = millis();
  while ((millis() - armtime) < 35000)
  {
    if (panicking == true);
    {
      break;
    }
    scoopstate = 5;
    if (scoopstate == scoopstatemem)
    {
      Serial.println(scoopstatemem);
      return;
    } else {
      scoopstatemem = scoopstate;
      scpcontrol();
    }
  }
  while ((millis() - armtime) < 20000)
  {
    if (panicking == true);
    {
      break;
    }
    scoopstate = 3;
    if (scoopstate == scoopstatemem)
    {
      Serial.println(scoopstatemem);
      return;
    } else {
      scoopstatemem = scoopstate;
      scpcontrol();
    }
  }
  scoopstatemem = 0;
  scpcontrol();
  scoopraised = true;

  // if in automatic mode, reset after depositing
  if (scoopmode == true)
  {
    Serial.println("Resetting after depositing");
    resetScoop();
  }
}

// reset to neutral position
void resetScoop()
{
  Serial.println("resetscoop called");
  // if already reset, return
  if (scoopdigging == false && scoopraised == false)
  {
    Serial.println("Already reset");
    return;
  }

  // if raised, lower to reset
  if (scoopdigging == false && scoopraised == true)
  {
    Serial.println("Raised, resetting");
    // 55 seconds for forearm, 35 seconds for bicep
    armtime = millis();
    while ((millis() - armtime) < 35000)
    {
      if (panicking == true);
      {
        break;
      }
      scoopstate = 6;
      if (scoopstate == scoopstatemem)
      {
        Serial.println(scoopstatemem);
        return;
      } else {
        scoopstatemem = scoopstate;
        scpcontrol();
      }
    }

    while ((millis() - armtime) < 20000)
    {
      if (panicking == true);
      {
        break;
      }
      scoopstate = 4;
      if (scoopstate == scoopstatemem)
      {
        Serial.println(scoopstatemem);
        return;
      } else {
        scoopstatemem = scoopstate;
        scpcontrol();
      }
    }
    scoopstatemem = 0;
    scpcontrol();
    scoopraised = false;
    return;
  }

  // if digging, return to neutral
  if (scoopdigging == true && scoopraised == false)
  {
    Serial.println("Digging, resetting");
    // placeholder for this code section
    delay(1);

    scoopdigging = false;
    return;
  }
}
