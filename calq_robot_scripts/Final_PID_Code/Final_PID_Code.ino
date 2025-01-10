
// Define constants for pulses per revolution (PPR) and gear ratio (GR)
const float PPR = 3575.0855;
const float GR = 297.924;
const float CPR = 3;

// Variables for tracking encoder positions
volatile long counter_m1 = 0;
volatile long counter_m2 = 0;
int aLastState_m1;
int aLastState_m2;

// Pins for reading encoders of motor 1 and 2
const int encoderPinA_m1 = 2;
const int encoderPinB_m1 = 10;
const int encoderPinA_m2 = 3;
const int encoderPinB_m2 = 11;

// Pins for setting the direction of motor 1 and 2
const int motorPin1_m1 = 4;
const int motorPin2_m1 = 5;
const int motorPin1_m2 = 7;
const int motorPin2_m2 = 8;


// Pins for setting the speed of rotation (Enable pin) of motors 1 and 2
const int enablePin_m1 = 6;
const int enablePin_m2 = 9;
int receive_data = 1;

// Variables for encoder positions and desired positions
long currentPosition_m1 = 0;
long currentPosition_m2 = 0;
float demandPositionInDegrees_m1;
float demandPositionInDegrees_m2;
float currentPositionInDegrees_m1;
float currentPositionInDegrees_m2;

// Time parameters
unsigned long currentTime;
unsigned long previousTime = 0;
unsigned long deltaT;

// PID gains
float Kp_m1 =2.5, Kd_m1 = 0.00, Ki_m1 = 0.000;
// float Kp_m2 = 1, Kd_m2 = 0.07, Ki_m2 = 0.017;
float Kp_m2 =2, Kd_m2 = 0.001, Ki_m2 = 0.0001;
// 1.2, 0.002, 0.012

// Error values
float errorPositionInDegrees_prev_m1 = 0, errorPositionInDegrees_sum_m1 = 0;
float errorPositionInDegrees_prev_m2 = 0, errorPositionInDegrees_sum_m2 = 0;


void setup() {
  Serial.begin(115200);

  // Task 1: Initialize the pins using pinMode and attachInterrupt functions

  // Set the motor pins as outputs
  pinMode(motorPin1_m1, OUTPUT);
  pinMode(motorPin2_m1, OUTPUT);
  pinMode(motorPin1_m2, OUTPUT);
  pinMode(motorPin2_m2, OUTPUT);

  // Set the enable pins as outputs
  pinMode(enablePin_m1, OUTPUT);
  pinMode(enablePin_m2, OUTPUT);

  // Set the encoder pins as inputs
  pinMode(encoderPinA_m1, INPUT_PULLUP);
  pinMode(encoderPinB_m1, INPUT_PULLUP);
  pinMode(encoderPinA_m2, INPUT_PULLUP);
  pinMode(encoderPinB_m2, INPUT_PULLUP);

  // Attach interrupts to the encoder pins to track motor positions
  attachInterrupt(digitalPinToInterrupt(encoderPinA_m1), updateEncoder_m1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinA_m2), updateEncoder_m2, CHANGE);

  aLastState_m1 = digitalRead(encoderPinA_m1);
  aLastState_m2 = digitalRead(encoderPinA_m2);

  delay(3000);
  previousTime = micros();
}


void loop() {
  currentTime = micros();
  deltaT = currentTime - previousTime;  // Task 3: Compute elapsed time
  previousTime = currentTime;
  Serial.println(deltaT);
  
  // if (receive_data == 1) {
  if (Serial.available() > 0) {
    receive_data = 0;
    String command = Serial.readStringUntil('\n');  // Read the incoming command
    
    // Check if the command starts with 'C' to update demand positions
    if (command.startsWith("C")) {
      int commaIndex = command.indexOf(',');
      if (commaIndex > 1) {
        // Parse the angles
        float angle1 = command.substring(1, commaIndex).toFloat();
        float angle2 = command.substring(commaIndex + 1).toFloat();
        
        // Update demand positions
        demandPositionInDegrees_m1 = angle1;
        demandPositionInDegrees_m2 = angle2;
        
        Serial.print("Updated Demand: ");
        Serial.print(angle1);
        Serial.print(", ");
        Serial.println(angle2);
      }
    }
  }
  

//  demandPositionInDegrees_m1 = 90;
//  demandPositionInDegrees_m2 = -90;

  // Task 2: Compute the current position in degrees and bound it to [-360, 360]
  currentPositionInDegrees_m1 = ((counter_m1 * 360.0) / (CPR * GR * 5));
  currentPositionInDegrees_m2 = ((counter_m2 * 360.0) / (CPR * GR * 5));

  // Bound the computed degrees within [-360, 360]
  if (currentPositionInDegrees_m1 > 360.0 || currentPositionInDegrees_m1 <= -360.0) {
    counter_m1 -= ((GR * CPR * 2) * (int)(currentPositionInDegrees_m1 / 360));
  }
  if (currentPositionInDegrees_m2 > 360.0 || currentPositionInDegrees_m2 <= -360.0) {
    counter_m2 -= ((GR * CPR * 2) * (int)(currentPositionInDegrees_m2 / 360));
  }

  // Serial.println(deltaT);
  if (deltaT > 400) {
    // Task 4: Compute PID errors for Motor 1 and Motor 2
    float errorPositionInDegrees_m1 = demandPositionInDegrees_m1 - currentPositionInDegrees_m1;
    float errorPositionInDegrees_m2 = demandPositionInDegrees_m2 - currentPositionInDegrees_m2;

    // Integral error accumulation
    errorPositionInDegrees_sum_m1 += errorPositionInDegrees_m1;
    errorPositionInDegrees_sum_m2 += errorPositionInDegrees_m2;

    // Derivative error
    float errorPositionInDegrees_diff_m1 = (errorPositionInDegrees_m1 - errorPositionInDegrees_prev_m1) / deltaT;
    float errorPositionInDegrees_diff_m2 = (errorPositionInDegrees_m2 - errorPositionInDegrees_prev_m2) / deltaT;

    // Task 5: Compute PID output for Motor 1 and Motor 2
    float output_m1 = (Kp_m1 * errorPositionInDegrees_m1) + (Ki_m1 * errorPositionInDegrees_sum_m1) + (Kd_m1 * errorPositionInDegrees_diff_m1);
    float output_m2 = (Kp_m2 * errorPositionInDegrees_m2) + (Ki_m2 * errorPositionInDegrees_sum_m2) + (Kd_m2 * errorPositionInDegrees_diff_m2);

// Check if m1 and m2 are going clockwise or counterclockwise and constrain accordingly

if (output_m1 > 0) {  // m1 is going clockwise
    output_m1 = constrain(output_m1, 50, 255);
} else if (output_m1 < 0) {  // m1 is going anti-clockwise
    output_m1 = constrain(output_m1, -255, -54);
} else {  // m1 is stopped
    output_m1 = 0;
}

if (output_m2 > 0) {  // m2 is going clockwise
    output_m2 = constrain(output_m2, 60, 255);
} else if (output_m2 < 0) {  // m2 is going anti-clockwise
    output_m2 = constrain(output_m2, -255, -60);
} else {  // m2 is stopped
    output_m2 = 0;
}


    // Task 6: Send voltage to motors
    if (output_m1 >= 0) {
      digitalWrite(motorPin1_m1, HIGH);
      digitalWrite(motorPin2_m1, LOW);
      analogWrite(enablePin_m1, output_m1);
    } else {
      digitalWrite(motorPin1_m1, LOW);
      digitalWrite(motorPin2_m1, HIGH);
      analogWrite(enablePin_m1, -output_m1);
    }


    if (output_m2 >= 0) {
      digitalWrite(motorPin1_m2, HIGH);
      digitalWrite(motorPin2_m2, LOW);
      analogWrite(enablePin_m2, output_m2);
    } else {
      digitalWrite(motorPin1_m2, LOW);
      digitalWrite(motorPin2_m2, HIGH);
      analogWrite(enablePin_m2, -output_m2);
    }


    // Task 4: Update previous errors
    errorPositionInDegrees_prev_m1 = errorPositionInDegrees_m1;
    errorPositionInDegrees_prev_m2 = errorPositionInDegrees_m2;

  // Task 7: Plot current and demand positions
    // Control the frequency of printing
  Serial.print("Motor1_Demand:");
  Serial.print(demandPositionInDegrees_m1);
  Serial.print(" Motor1_Current:");
  Serial.print(currentPositionInDegrees_m1);
  Serial.print(" Motor2_Demand:");
  Serial.print(demandPositionInDegrees_m2);
  Serial.print(" Motor2_Current:");
  Serial.println(currentPositionInDegrees_m2);


    // Serial.print(demandPositionInDegrees_m1);  // Print Motor 1 Demand
    // Serial.print(",");            // Separate values with a comma
    // Serial.print(currentPositionInDegrees_m1);  // Print Motor 1 Current
    // Serial.print(",");            // Separate values with a comma
    // Serial.print(demandPositionInDegrees_m2);   // Print Motor 2 Demand
    // Serial.print(",");            // Separate values with a comma
    // Serial.println(currentPositionInDegrees_m2);  // Print Motor 2 Current and end the line


    delay(100);

    
  }
}


// // Interrupt functions for tracking the encoder positions
// void updateEncoder_m1() {
//   // Code to update counter_m1 based on the state of the encoder pins
// }

// void updateEncoder_m2() {
//   // Code to update counter_m2 based on the state of the encoder pins
// }


// Function to update encoder for motor 1
void updateEncoder_m1() {
  int currentState_m1 = digitalRead(encoderPinA_m1);
  if (currentState_m1 != aLastState_m1) {
    if (digitalRead(encoderPinB_m1) != currentState_m1) {
      counter_m1++;
    } else {
      counter_m1--;
    }
    aLastState_m1 = currentState_m1;
  }
  
}
 
// Function to update encoder for motor 2
void updateEncoder_m2() {
  int currentState_m2 = digitalRead(encoderPinA_m2);
  if (currentState_m2 != aLastState_m2) {
    if (digitalRead(encoderPinB_m2) != currentState_m2) {
      counter_m2++;
    } else {
      counter_m2--;
    }
    aLastState_m2 = currentState_m2;
  }
  
}
