/* ************************************************************************************************* */
// UCSD ECE 5 Lab 4 Code: Line Following Robot with PID 
// V 4.0
// Last Modified 5/14/2025 by MingWei Yeoh, Korey Huynh, and Karcher Morris
/* ************************************************************************************************* */

/*
   This is code for your PID controlled line following robot.

   ******      Code Table of Contents      ******

  - Line_Follower_Code_Basic
   > Declare libraries     - declares global variables so each variable can be accessed from every function
   > Declare Pins          - where the user sets what pin everything is connected to 
   > Settings              - settings that can improve robot functionality and help to debug
   > Setup (Main)          - runs once at beginning when you press button on arduino or when you open serial monitor
   > Loop  (Main)          - loops forever calling on a series of function
   
  - Calibration 
   > Main Calibrate()      - runs calibration function calls and synchronizes calibration state with different led animations
  
  - Helper_Functions
   > setLEDs               - turns on all LEDs in the LED_Pin array on or off
   > Read Potentiometers   - reads each potentiometer
   > Read Photoresistors   - reads each photoresistor
   > Run Motors            - runs motors
   > Calculate Error       - calculate error from photoresistor readings
   > PID Turn              - takes the error and implements PID control
   > Print                 - used for printing information but should disable when not debugging because it slows down program

*/

// Include files needed
#include <L298NX2.h> // Using "L298N" library found through arduino library manager developed by Andrea Lombardo (https://github.com/AndreaLombardo/L298N)

// ************************************************************************************************* //
// ************************************************************************************************* //
// Change Robot Settings here

#define PRINTALLDATA        1  // 1 = send debug data (gains, sensors, error, motor speeds). Safe to leave on untethered:
                                //     it is rate-limited and never blocks the PID loop.
#define USB_PRINT           1  // 1 = also print to the USB Serial Monitor / Serial Plotter (when plugged in)
#define BLE_TELEMETRY       1  // 1 = also send wirelessly over Bluetooth LE (see BleTelemetry.h / ble_viewer.html)
                                //     0 = Bluetooth is not compiled in at all
#define TELEMETRY_PERIOD_MS 50 // send one line every 50 ms (20 lines/sec). The PID loop itself still runs at full speed.
#define BLE_DEVICE_NAME     "ECE5-Robot" // name you'll see when connecting; change it so teams don't collide
#define NOMINALSPEED        50 // This is the base speed for both motors, can also be increased by using potentiometers

#if BLE_TELEMETRY
#include "BleTelemetry.h"      // Wireless debug output (uses the ESP32 core's built-in BLE library, nothing to install)
#endif

// ************************************************************************************************* //

// ****** DECLARE PINS HERE  ****** 

// Taken from LEFT TO RIGHT of the robot ****** Orient yourself so that you are looking from the rear of the robot (photoresistors are farthest away from you, wheels are closest to you)
//                  Left Motors   Right motors 
L298NX2 DriveMotors(42, 41, 40,      37, 39, 38);
//                 ENA, IN1, IN2, ENB, IN3, IN4

enum side {LEFT, RIGHT};

int LDR_Pin[] = {1,2,3,4,5,6,7}; // SET PINS CONNECTED TO PHOTORESISTORS // FROM LEFT TO RIGHT OF THE ROBOT, ROBOT IS ORIENTED WHERE PHOTORESISOTRS FARTHEST FROM YOU AND WHEELS ARE CLOSEST TO YOU      

// Potentiometer Pins
const int S_pin = 13; // Pin connected to Speed potentiometer
const int P_pin = 12; // Pin connected to P term potentiometer
const int I_pin = 11; // Pin connected to I term potentiometer
const int D_pin = 10; // Pin connected to D term potentiometer
                                                                 
int led_Pins[] = {46};  // LEDs to indicate what part of calibration you're on and to illuminate the photoresistors

// ****** DECLARE Variables HERE  ****** 

//Variables Potentiometer Reading
int SpRead = 0; // speed increase
int kPRead = 0; // proportional gain
int kIRead = 0; // integral gain
int kDRead = 0; // derivative gain

// Variables for Calibration and Error Calculation
float Mn[20]; 
float Mx[20];
float LDRf[20];
int LDR[20];    
int rawPResistorData[20];  
int totalPhotoResistors = sizeof(LDR_Pin) / sizeof(LDR_Pin[0]);  
int numLEDs = sizeof(led_Pins) / sizeof(led_Pins[0]); 
int MxRead, MxIndex, CriteriaForMax;
int leftHighestPR, highestPResistor, rightHighestPR;
float AveRead, WeightedAve;   

// For Motor Control
int M1SpeedtoMotor, M2SpeedtoMotor;
int Turn, M1P = 0, M2P = 0;
float error, lasterror = 0, sumerror = 0;
float kP, kI, kD;

// For Debug Output
char telemetryLine[200];   // one formatted line of debug data (fixed buffer, no String = fast & no heap churn)
char calInfo[3][160];      // White / Black / Delta calibration lines, re-sent whenever Bluetooth connects



// ************************************************************************************************* //
// setup - runs once
void setup() {
  Serial.begin(9600);                            // For serial communication set up
#if ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0);                      // never wait on USB when no computer is plugged in
#endif

#if BLE_TELEMETRY
  bleTelemetryBegin(BLE_DEVICE_NAME);            // start advertising now so you can connect during calibration
#endif

  for (int i = 0; i < numLEDs; i++)
    pinMode(led_Pins[i], OUTPUT);                // Initialize all LEDs to output
  
  Calibrate();                                   // Calibrate black and white sensing

  ReadPotentiometers();                          // Read potentiometer values (Sp, P, I, & D)

} // end setup()

// ************************************************************************************************* //
// loop - runs/loops forever
void loop() {

  ReadPotentiometers(); // Read potentiometers

  ReadPhotoResistors(); // Read photoresistors 

  CalcError();          // Calculates error

  PID_Turn();           // PID Control and Output to motors to turn

  RunMotors();          // Runs motors
  
  if (PRINTALLDATA)     // If PRINTALLDATA Enabled, send the data (rate-limited, non-blocking)
    Print();

  
  
  
} // end loop()






// ************************************************************************************************* //
// function to calibrate

void Calibrate() {

  int numberOfMeasurements = 20;                // set number Of Measurements to take

  CalibrateHelper(numberOfMeasurements, false); // White Calibration

  runMotorAtSpeed(LEFT, 80);          // Run both wheels forward
  runMotorAtSpeed(RIGHT, 80);
  delay(2000);                                  // for 1 second
  DriveMotors.stopA();                          // then stop both motors
  DriveMotors.stopB();


  setLeds(0);                                  // Turn off LEDs to indicate user to calibrate other color
  delay(2000);
  
  CalibrateHelper(numberOfMeasurements, true);  // Black Calibration

  
  runMotorAtSpeed(LEFT, 80);          // Run both wheels forward
  runMotorAtSpeed(RIGHT, 80);
  delay(2000);                                  // for 1 second
  DriveMotors.stopA();                          // then stop both motors
  DriveMotors.stopB();

  // Build the White / Black / Delta lines once, print them, and keep them so they can be
  // re-sent over Bluetooth when you connect (calibration usually finishes before you connect).
  const char *calLabels[3] = {"White Vals:  ", "Black Vals:  ", "Delta Vals:  "};
  for (int row = 0; row < 3; row++) {
    int len = snprintf(calInfo[row], sizeof(calInfo[row]), "%s", calLabels[row]);
    for (int i = 0; i < totalPhotoResistors && len < (int)sizeof(calInfo[row]); i++) {
      float v = (row == 0) ? Mn[i] : (row == 1) ? Mx[i] : (Mx[i] - Mn[i]); // White, Black, or Difference
      len += snprintf(calInfo[row] + len, sizeof(calInfo[row]) - len, "%.0f ", v);
    }
    if (len < (int)sizeof(calInfo[row]) - 1)
      strcat(calInfo[row], "\n");
    DebugMsg(calInfo[row]);
  }

  setLeds(1);                                   // Turn LEDs on
  delay(2000);

} // end Calibrate()

void CalibrateHelper(int numberOfMeasurements, boolean ifCalibratingBlack) {
  
  if (ifCalibratingBlack)
    DebugMsg("\nCalibrating Black\n");
  else
    DebugMsg("\nCalibrating White\n");
// Indicate that calibration is starting
  for (int i = 0; i < 4; i++) {
      setLeds(1); // turn the LEDs on
      delay(250); // wait
      setLeds(0); // turn the LEDs off
      delay(250); // wait 
  }
  
  setLeds(1);
  delay(250);

  for (int i = 0; i < numberOfMeasurements; i++) {
    for (int pin = 0; pin < totalPhotoResistors; pin++) {
      LDRf[pin] = LDRf[pin] + (float)analogRead(LDR_Pin[pin]);
      delay(2);
    }
    Serial.print(". ");
  }
  for (int pin = 0; pin < totalPhotoResistors; pin++) {
    if (ifCalibratingBlack) {                                   // updating cooresponding array based on if we are calibrating black or white
      Mx[pin] = round(LDRf[pin] / (float)numberOfMeasurements); // take average and store for black
    }
    else {
      Mn[pin] = round(LDRf[pin] / (float)numberOfMeasurements); // take average and store for white
    }
    LDRf[pin] = 0.0;
  }

  DebugMsg(" Done!\n");
  setLeds(0);
  delay(250);
}

// Set all LEDs to a certain brightness
void setLeds(int x) {
  for (int i = 0; i < numLEDs; i++)
    digitalWrite(led_Pins[i], x);
}

// **********Recall your Challenge #1 Code********************************************************************** //
// function to read and map values from potentiometers
void ReadPotentiometers() {
  // Call on user-defined function to read Potentiometer values
  SpRead = ReadPotentiometerHelper(S_pin, 0, 4095, 0, 100); // We want to read a potentiometer for S_pin with resolution from 0 to 1023 and potentiometer range from 0 to 100.
  kPRead = ReadPotentiometerHelper(P_pin, 0, 4095, 0, 100); // We want to read a potentiometer for P_pin with resolution from 0 to 1023 and potentiometer range from 0 to 100.
  kIRead = ReadPotentiometerHelper(I_pin, 0, 4095, 0, 100); // We want to read a potentiometer for I_pin with resolution from 0 to 1023 and potentiometer range from 0 to 100.
  kDRead = ReadPotentiometerHelper(D_pin, 0, 4095, 0, 100); // We want to read a potentiometer for D_pin with resolution from 0 to 1023 and potentiometer range from 0 to 100.

} // end ReadPotentiometers()

int ReadPotentiometerHelper(int pin, int min_resolution, int max_resolution, int min_potentiometer, int max_potentiometer) {
  return map(analogRead(pin), min_resolution, max_resolution, min_potentiometer, max_potentiometer); 
}

// **********Recall your Challenge #2 Code********************************************************************** //
// Function to read photo resistors and map from 0 to 100
void ReadPhotoResistors() {
  for (int i = 0; i < totalPhotoResistors; i++) { 
    rawPResistorData[i] = analogRead(LDR_Pin[i]);
    LDR[i] = map(rawPResistorData[i], Mn[i], Mx[i], 0, 100); // Mn and Mx are created from calibration Min and Max for each pin
  }    

} // end ReadPhotoResistors()


// **********Recall your Challenge #3 Code********************************************************************** //
// function to start motors using nominal speed + speed addition from potentiometer
void RunMotors() {
  M1SpeedtoMotor = min(NOMINALSPEED + SpRead + M1P, 255); // limits speed to 255
  M2SpeedtoMotor = min(NOMINALSPEED + SpRead + M2P, 255); // remember M1Sp & M2Sp is defined at beginning of code (default 60)
  
  runMotorAtSpeed(LEFT, M2SpeedtoMotor); // run right motor
  runMotorAtSpeed(RIGHT, M1SpeedtoMotor); // run left motor
} // end RunMotors()

// A function that commands a specified motor to move towards a given direction at a given speed
void runMotorAtSpeed(side _side, int speed) {
  if (_side == LEFT) {
    DriveMotors.setSpeedA(abs(speed));
    if (speed > 0)                // swap direction if speed is negative
      DriveMotors.backwardA();          // sets the direction of the motor from arguments
    else
      DriveMotors.forwardA();           // sets the direction of the motor from arguments
  }
  if (_side == RIGHT) {
    DriveMotors.setSpeedB(abs(speed));
    if (speed > 0)                // swap direction if speed is negative
      DriveMotors.backwardB();          // sets the direction of the motor from arguments
    else
      DriveMotors.forwardB();           // sets the direction of the motor from arguments
  }
}



// ************************************************************************************************* //
// Calculate error from photoresistor readings
void CalcError() {
  MxRead = -99;
  AveRead = 0.0;
  for (int i = 0; i < totalPhotoResistors; i++) { // This loop goes through each photoresistor and stores the photoresistor with the highest value to the variable 'highestPResistor'
    if (MxRead < LDR[i]) {
      MxRead = LDR[i];
      MxIndex = -1 * (i - 3);
      highestPResistor = (float)i;
    }
    AveRead = AveRead + (float)LDR[i] / (float)totalPhotoResistors;
  }
  
  CriteriaForMax = 1.5; 
  if (MxRead > CriteriaForMax * AveRead) { // Make sure that the highestPResistor is actually "seeing" a line. What happens if there is no line and we take the photoresistor that happens to have the highest value?

    // Next we assign variables to hold the index of the left and right Photoresistor that has the highest value, though we have to make sure that we aren't checking a Photoresistor that doesn't exist.
    // Ex: To the left of the left most photoresistor or the right of the right most photoresistor
    if (highestPResistor != 0)
      leftHighestPR = highestPResistor - 1;
    else
      leftHighestPR = highestPResistor;

    if (highestPResistor != totalPhotoResistors - 1)
      rightHighestPR = highestPResistor + 1;
    else
      rightHighestPR = highestPResistor;

    // Next we take the percentage of "line" each of our left, middle, and right photoresistors sees and then we take the average, which is our error calculation
    float numerator = (float)(LDR[leftHighestPR] * leftHighestPR) + (float)(LDR[highestPResistor] * highestPResistor) + (float)(LDR[rightHighestPR] * rightHighestPR);
    float denominator = (float)LDR[leftHighestPR] + (float)LDR[highestPResistor] + (float)LDR[rightHighestPR];

    WeightedAve = ((float)numerator) / denominator;

    error = (WeightedAve - totalPhotoResistors/2);
  }
  
} // end CalcError()

// ************************************************************************************************* //
// PID Function
void PID_Turn() {
  kP = (float)kPRead * 1.;    // each of these scaling factors can change depending on how influential you want them to be
  kI = (float)kIRead * 0.001;
  kD = (float)kDRead * 0.01;

  Turn = error * kP + sumerror * kI + (error - lasterror) * kD; // PID!!!!!!!!!!!!!

  if (sumerror > 5)   // prevents integrator wind-up
    sumerror = 5;
  else if (sumerror < -5)
    sumerror = -5;

  if (error == 0)     // Reset sumerror if line is centered
    sumerror = 0;

  if (Turn < 0) {
    M1P = -Turn;       // One motor becomes slower and the other faster
    M2P = Turn;
  }
  else if (Turn > 0) {
    M1P = -Turn;
    M2P = Turn;
  }
  else {
    M1P = 0;
    M2P = 0;
  }

  lasterror = error;
  sumerror = sumerror + error;

} // end PID_Turn()

// ************************************************************************************************* //
// Send a one-off message (calibration status, etc.) to USB and/or Bluetooth
void DebugMsg(const char *msg) {
#if USB_PRINT
  Serial.print(msg);
#endif
#if BLE_TELEMETRY
  bleSend(msg);
#endif
}

// ************************************************************************************************* //
// function to print values of interest
// Rate-limited by a millis() timer instead of delay(): the PID loop keeps running at full speed and
// only one line is sent every TELEMETRY_PERIOD_MS. Format is "label:value" pairs, so the Arduino
// Serial Plotter (USB) and ble_viewer.html (Bluetooth) can both graph it.
void Print() {
  static unsigned long lastSend = 0;

#if BLE_TELEMETRY
  if (bleConsumeNewConnection())                 // just connected? re-send the calibration results first
    for (int row = 0; row < 3; row++)
      bleSend(calInfo[row]);
  bool bleWantsData = bleConnected();
#else
  bool bleWantsData = false;
#endif

  if (!USB_PRINT && !bleWantsData)               // nobody listening, so skip the formatting work entirely
    return;

  unsigned long now = millis();
  if (now - lastSend < TELEMETRY_PERIOD_MS)      // not time yet, return immediately
    return;
  lastSend = now;

  int len = snprintf(telemetryLine, sizeof(telemetryLine), "Sp:%d,P:%.2f,I:%.3f,D:%.2f", SpRead, kP, kI, kD); // PID settings

  for (int i = 0; i < totalPhotoResistors && len < (int)sizeof(telemetryLine); i++)  // photoresistor values, L0 = leftmost
    len += snprintf(telemetryLine + len, sizeof(telemetryLine) - len, ",L%d:%d", i, LDR[i]);
    // use rawPResistorData[i] instead of LDR[i] if you would prefer to see raw photoresistor readings

  if (len < (int)sizeof(telemetryLine))          // error (-3 through 3) and motor outputs (0-255)
    len += snprintf(telemetryLine + len, sizeof(telemetryLine) - len, ",Err:%.2f,LMotor:%d,RMotor:%d\n",
                    error, M1SpeedtoMotor, M2SpeedtoMotor);
  if (len >= (int)sizeof(telemetryLine))
    len = sizeof(telemetryLine) - 1;

#if USB_PRINT
  Serial.write((const uint8_t *)telemetryLine, len);
#endif
#if BLE_TELEMETRY
  if (bleWantsData)
    bleSend(telemetryLine, len);
#endif

} // end Print()
