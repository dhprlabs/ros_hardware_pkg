#include <Arduino.h>

// === Encoder Pins ===
const int encoderPinAA = 2;   // Encoder 1 - Channel A
const int encoderPinAB = 4;   // Encoder 1 - Channel B
const int encoderPinBA = 12;  // Encoder 2 - Channel A
const int encoderPinBB = 13;  // Encoder 2 - Channel B

// === Motor PWM Pins ===
const int motor1_pwmA = 26;   // Motor 1 - PWM A
const int motor1_pwmB = 25;   // Motor 1 - PWM B
const int motor2_pwmA = 14;   // Motor 2 - PWM A
const int motor2_pwmB = 27;   // Motor 2 - PWM B

// === Encoder Counts ===
volatile long encoder1_count = 0;
volatile long encoder2_count = 0;

// === PID Control Variables ===
// Motor 1 PID
float motor1_target_speed = 0;  // Target speed in RPM or encoder counts/sec
float motor1_prev_error = 0;
float motor1_integral = 0;
long motor1_prev_count = 0;
unsigned long motor1_prev_time = 0;

// Motor 2 PID  
float motor2_target_speed = 0;  // Target speed in RPM or encoder counts/sec
float motor2_prev_error = 0;
float motor2_integral = 0;
long motor2_prev_count = 0;
unsigned long motor2_prev_time = 0;

// PID Constants - tune these values
float kp = 0.75;
float ki = 2.70; 
float kd = 0.00;

// Speed calculation
float motor1_current_speed = 0;
float motor2_current_speed = 0;

// Control loop timing
unsigned long control_loop_time = 0;
const unsigned long CONTROL_PERIOD = 50; // 50ms = 20Hz control loop

// Encoder specifications
const int COUNTS_PER_REV = 255;  // Encoder counts per revolution
const float MAX_REV_SPEED = 9.5; // Maximum revolutions per second

// Menu system variables
bool menu_mode = true;
String serial_input = "";
unsigned long last_feedback_time = 0;
const unsigned long FEEDBACK_PERIOD = 500; // Send feedback every 500ms

// ROS Communication variables
bool ros_mode = false;
bool continuous_stream = false; // Control continuous status streaming
unsigned long last_ros_command_time = 0;
const unsigned long ROS_TIMEOUT = 2000; // 2 seconds timeout
unsigned long last_ros_status_time = 0;
const unsigned long ROS_STATUS_PERIOD = 100; // Send status every 100ms in ROS mode

void IRAM_ATTR readEncoder1A() {
  int b = digitalRead(encoderPinAB);
  encoder1_count += (b > 0) ? -1 : 1;  // Inverted direction for Motor 1
}

void IRAM_ATTR readEncoder2A() {
  int b = digitalRead(encoderPinBB);
  encoder2_count += (b > 0) ? 1 : -1;
}


void setup() {
  Serial.begin(115200);
  Serial.println("=== Dual Motor PID Controller ===");
  Serial.println("Encoder: 255 counts/rev, Max: 9.5 rev/s");

  // Encoder pins
  pinMode(encoderPinAA, INPUT);
  pinMode(encoderPinAB, INPUT);
  pinMode(encoderPinBA, INPUT);
  pinMode(encoderPinBB, INPUT);

  // Setup PWM for motors
  const int freq = 1000;
  const int resolution = 8;
  ledcAttach(motor1_pwmA, freq, resolution);
  ledcAttach(motor1_pwmB, freq, resolution);
  ledcAttach(motor2_pwmA, freq, resolution);
  ledcAttach(motor2_pwmB, freq, resolution);

  // Initialize motors to stop
  stopMotors();
  
  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(encoderPinAA), readEncoder1A, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPinBA), readEncoder2A, RISING);
  
  // Initialize timing
  control_loop_time = millis();
  last_feedback_time = millis();
  motor1_prev_time = micros();
  motor2_prev_time = micros();
  
  // Show menu
  showMenu();
}

void loop() {
  // Check for serial commands
  checkSerialCommands();
  
  // ROS timeout safety check
  if (ros_mode && (millis() - last_ros_command_time > ROS_TIMEOUT)) {
    // No ROS command received for 2 seconds - safety stop
    stopAllMotors();
    // Serial.println("ROS_TIMEOUT");
  }
  
  // Run control loop at fixed interval
  if (millis() - control_loop_time >= CONTROL_PERIOD) {
    control_loop_time = millis();
    
    // Calculate current speeds
    calculateSpeeds();
    
    // Run PID control for both motors
    pidControlMotor1();
    pidControlMotor2();
  }
  
  // Send feedback based on mode
  if (ros_mode) {
    // Send ROS status updates - either continuous stream or on request
    if (continuous_stream && (millis() - last_ros_status_time >= ROS_STATUS_PERIOD)) {
      last_ros_status_time = millis();
      sendROSStatus();
    }
  } else {
    // Send periodic feedback in menu mode
    if (millis() - last_feedback_time >= FEEDBACK_PERIOD) {
      last_feedback_time = millis();
      sendFeedback();
    }
  }
}void showMenu() {
  Serial.println("\n=== MOTOR CONTROL MENU ===");
  Serial.println("1. Set Motor 1 Speed (rev/s)");
  Serial.println("2. Set Motor 2 Speed (rev/s)");
  Serial.println("3. Set Both Motors Speed");
  Serial.println("4. Stop All Motors");
  Serial.println("5. Tune PID Parameters");
  Serial.println("6. Show Current Status");
  Serial.println("7. Reset Encoders");
  Serial.println("8. Test 1 rev/s for 10 seconds");
  Serial.println("9. Show This Menu");
  Serial.println("10. Debug Encoder Readings");
  Serial.println("11. Test Motor 1 Direction");
  Serial.println("12. Test Motor 2 Direction");
  Serial.println("13. Enter ROS Mode");
  Serial.println("Enter command number:");
}

void checkSerialCommands() {
  if (Serial.available() > 0) {
    serial_input = Serial.readStringUntil('\n');
    serial_input.trim();
    
    if (ros_mode) {
      parseROSCommand();
    } else {
      parseCommand();
    }
  }
}

void parseROSCommand() {
  // ROS command format: "CMD_VEL <motor1_speed> <motor2_speed>"
  // or "EXIT_ROS" to return to menu mode
  
  if (serial_input.startsWith("CMD_VEL")) {
    last_ros_command_time = millis(); // Update timeout timer
    
    int first_space = serial_input.indexOf(' ');
    int second_space = serial_input.indexOf(' ', first_space + 1);
    
    if (first_space != -1 && second_space != -1) {
      float motor1_speed = serial_input.substring(first_space + 1, second_space).toFloat();
      float motor2_speed = serial_input.substring(second_space + 1).toFloat();
      
      // Apply speed limits
      motor1_speed = constrain(motor1_speed, -MAX_REV_SPEED, MAX_REV_SPEED);
      motor2_speed = constrain(motor2_speed, -MAX_REV_SPEED, MAX_REV_SPEED);
      
      // Set target speeds
      motor1_target_speed = motor1_speed * COUNTS_PER_REV;
      motor2_target_speed = motor2_speed * COUNTS_PER_REV;
      
      // Send acknowledgment
      Serial.println("ACK_CMD_VEL");
    } else {
      Serial.println("ERROR_INVALID_FORMAT");
    }
  }
  else if (serial_input == "START_STREAM") {
    continuous_stream = true;
    last_ros_command_time = millis(); // Update timeout timer
    Serial.println("ACK_START_STREAM");
  }
  else if (serial_input == "STOP_STREAM") {
    continuous_stream = false;
    Serial.println("ACK_STOP_STREAM");
  }
  else if (serial_input == "EXIT_ROS") {
    ros_mode = false;
    continuous_stream = false; // Stop streaming when exiting ROS mode
    stopAllMotors();
    Serial.println("Exiting ROS mode - returning to menu");
    showMenu();
  }
  else if (serial_input == "GET_STATUS") {
    sendROSStatus();
  }
  else if (serial_input == "STOP") {
    last_ros_command_time = millis(); // Update timeout timer
    stopAllMotors();
    Serial.println("ACK_STOP");
  }
  else {
    Serial.println("ERROR_UNKNOWN_COMMAND");
  }
}

void parseCommand() {
  int command = serial_input.toInt();
  
  switch (command) {
    case 1:
      Serial.println("Enter Motor 1 speed (rev/s, max 9.5):");
      waitForInput();
      setMotor1Speed(serial_input.toFloat());
      break;
      
    case 2:
      Serial.println("Enter Motor 2 speed (rev/s, max 9.5):");
      waitForInput();
      setMotor2Speed(serial_input.toFloat());
      break;
      
    case 3:
      Serial.println("Enter speed for both motors (rev/s, max 9.5):");
      waitForInput();
      setBothMotorsSpeeds(serial_input.toFloat());
      break;
      
    case 4:
      stopAllMotors();
      Serial.println("All motors stopped.");
      break;
      
    case 5:
      tunePIDParameters();
      break;
      
    case 6:
      showStatus();
      break;
      
    case 7:
      resetEncoders();
      Serial.println("Encoders reset to zero.");
      break;
      
    case 8:
      testOneRevPerSecond();
      break;
      
    case 9:
      showMenu();
      break;
      
    case 10:
      debugEncoders();
      break;
      
    case 11:
      testMotor1Direction();
      break;
      
    case 12:
      testMotor2Direction();
      break;
      
    case 13:
      enterROSMode();
      break;
      
    default:
      Serial.println("Invalid command. Enter 9 to show menu.");
      break;
  }
}

void waitForInput() {
  while (Serial.available() == 0) {
    delay(10);
  }
  serial_input = Serial.readStringUntil('\n');
  serial_input.trim();
}

void setMotor1Speed(float rev_per_sec) {
  if (rev_per_sec > MAX_REV_SPEED) {
    Serial.println("Warning: Speed limited to " + String(MAX_REV_SPEED) + " rev/s");
    rev_per_sec = MAX_REV_SPEED;
  } else if (rev_per_sec < -MAX_REV_SPEED) {
    Serial.println("Warning: Speed limited to " + String(-MAX_REV_SPEED) + " rev/s");
    rev_per_sec = -MAX_REV_SPEED;
  }
  
  motor1_target_speed = rev_per_sec * COUNTS_PER_REV; // Convert to counts/sec
  Serial.println("Motor 1 target speed set to " + String(rev_per_sec) + " rev/s (" + String(motor1_target_speed) + " counts/s)");
}

void setMotor2Speed(float rev_per_sec) {
  if (rev_per_sec > MAX_REV_SPEED) {
    Serial.println("Warning: Speed limited to " + String(MAX_REV_SPEED) + " rev/s");
    rev_per_sec = MAX_REV_SPEED;
  } else if (rev_per_sec < -MAX_REV_SPEED) {
    Serial.println("Warning: Speed limited to " + String(-MAX_REV_SPEED) + " rev/s");
    rev_per_sec = -MAX_REV_SPEED;
  }
  
  motor2_target_speed = rev_per_sec * COUNTS_PER_REV; // Convert to counts/sec
  Serial.println("Motor 2 target speed set to " + String(rev_per_sec) + " rev/s (" + String(motor2_target_speed) + " counts/s)");
}

void setBothMotorsSpeeds(float rev_per_sec) {
  setMotor1Speed(rev_per_sec);
  setMotor2Speed(rev_per_sec);
}

void stopAllMotors() {
  motor1_target_speed = 0;
  motor2_target_speed = 0;
  stopMotors();
}

void tunePIDParameters() {
  Serial.println("Current PID values: Kp=" + String(kp) + " Ki=" + String(ki) + " Kd=" + String(kd));
  
  Serial.println("Enter new Kp value:");
  waitForInput();
  kp = serial_input.toFloat();
  
  Serial.println("Enter new Ki value:");
  waitForInput();
  ki = serial_input.toFloat();
  
  Serial.println("Enter new Kd value:");
  waitForInput();
  kd = serial_input.toFloat();
  
  Serial.println("PID parameters updated: Kp=" + String(kp) + " Ki=" + String(ki) + " Kd=" + String(kd));
  
  // Reset integral terms when PID is changed
  motor1_integral = 0;
  motor2_integral = 0;
}

void showStatus() {
  Serial.println("\n=== CURRENT STATUS ===");
  Serial.println("PID Parameters: Kp=" + String(kp) + " Ki=" + String(ki) + " Kd=" + String(kd));
  Serial.println("Motor 1 Target: " + String(motor1_target_speed/COUNTS_PER_REV) + " rev/s (" + String(motor1_target_speed) + " counts/s)");
  Serial.println("Motor 1 Actual: " + String(motor1_current_speed/COUNTS_PER_REV) + " rev/s (" + String(motor1_current_speed) + " counts/s)");
  Serial.println("Motor 2 Target: " + String(motor2_target_speed/COUNTS_PER_REV) + " rev/s (" + String(motor2_target_speed) + " counts/s)");
  Serial.println("Motor 2 Actual: " + String(motor2_current_speed/COUNTS_PER_REV) + " rev/s (" + String(motor2_current_speed) + " counts/s)");
  Serial.println("Encoder 1 Count: " + String(encoder1_count));
  Serial.println("Encoder 2 Count: " + String(encoder2_count));
  Serial.println("===================\n");
}

void resetEncoders() {
  encoder1_count = 0;
  encoder2_count = 0;
  motor1_prev_count = 0;
  motor2_prev_count = 0;
}

void testOneRevPerSecond() {
  Serial.println("Testing 1 rev/s for 10 seconds on both motors...");
  long initial_count1 = encoder1_count;
  long initial_count2 = encoder2_count;
  
  setBothMotorsSpeeds(1.0); // Set to 1 rev/s
  
  unsigned long test_start = millis();
  unsigned long last_print = millis();
  
  while (millis() - test_start < 10000) { // Run for 10 seconds
    // Continue running the control loop
    if (millis() - control_loop_time >= CONTROL_PERIOD) {
      control_loop_time = millis();
      calculateSpeeds();
      pidControlMotor1();
      pidControlMotor2();
    }
    
    // Print status every second
    if (millis() - last_print >= 1000) {
      last_print = millis();
      float elapsed_seconds = (millis() - test_start) / 1000.0;
      long counts1 = encoder1_count - initial_count1;
      long counts2 = encoder2_count - initial_count2;
      float revs1 = counts1 / (float)COUNTS_PER_REV;
      float revs2 = counts2 / (float)COUNTS_PER_REV;
      
      Serial.println("Time: " + String(elapsed_seconds) + "s | M1: " + String(revs1) + " revs | M2: " + String(revs2) + " revs");
    }
  }
  
  // Final results
  long final_counts1 = encoder1_count - initial_count1;
  long final_counts2 = encoder2_count - initial_count2;
  float final_revs1 = final_counts1 / (float)COUNTS_PER_REV;
  float final_revs2 = final_counts2 / (float)COUNTS_PER_REV;
  
  Serial.println("\n=== TEST RESULTS ===");
  Serial.println("Expected: 10.0 revolutions each motor");
  Serial.println("Motor 1: " + String(final_revs1) + " revolutions (" + String(final_counts1) + " counts)");
  Serial.println("Motor 2: " + String(final_revs2) + " revolutions (" + String(final_counts2) + " counts)");
  Serial.println("Motor 1 Error: " + String(abs(10.0 - final_revs1)) + " revs (" + String(abs(10.0 - final_revs1)/10.0*100) + "%)");
  Serial.println("Motor 2 Error: " + String(abs(10.0 - final_revs2)) + " revs (" + String(abs(10.0 - final_revs2)/10.0*100) + "%)");
  Serial.println("==================\n");
  
  stopAllMotors();
}

void calculateSpeeds() {
  static unsigned long motor1_last_update = 0;
  static unsigned long motor2_last_update = 0;
  unsigned long current_time = micros();
  
  // Motor 1 speed calculation - update every 50ms minimum for stability
  if (current_time - motor1_last_update >= 50000) { // 50ms
    long motor1_count_diff = encoder1_count - motor1_prev_count;
    unsigned long motor1_time_diff = current_time - motor1_prev_time;
    
    if (motor1_time_diff > 0) {
      motor1_current_speed = (float)motor1_count_diff / (motor1_time_diff / 1000000.0); // counts per second
    }
    
    motor1_prev_count = encoder1_count;
    motor1_prev_time = current_time;
    motor1_last_update = current_time;
  }
  
  // Motor 2 speed calculation - update every 50ms minimum for stability
  if (current_time - motor2_last_update >= 50000) { // 50ms
    long motor2_count_diff = encoder2_count - motor2_prev_count;
    unsigned long motor2_time_diff = current_time - motor2_prev_time;
    
    if (motor2_time_diff > 0) {
      motor2_current_speed = (float)motor2_count_diff / (motor2_time_diff / 1000000.0); // counts per second
    }
    
    motor2_prev_count = encoder2_count;
    motor2_prev_time = current_time;
    motor2_last_update = current_time;
  }
}

void pidControlMotor1() {
  float error = motor1_target_speed - motor1_current_speed;
  motor1_integral += error * (CONTROL_PERIOD / 1000.0);
  float derivative = (error - motor1_prev_error) / (CONTROL_PERIOD / 1000.0);
  
  // Anti-windup for integral term
  if (motor1_integral > 50) motor1_integral = 50;
  if (motor1_integral < -50) motor1_integral = -50;
  
  float output = kp * error + ki * motor1_integral + kd * derivative;
  
  // Better output scaling - limit to reasonable range
  output = constrain(output, -200, 200);
  
  // Convert to PWM and direction
  int pwm_value = constrain(abs(output), 30, 255); // Minimum 30 for motor movement
  int direction = (output >= 0) ? 1 : -1;
  
  // Only apply PWM if target speed is not zero
  if (abs(motor1_target_speed) > 1.0) {
    setMotor(direction, pwm_value, motor1_pwmA, motor1_pwmB);
  } else {
    setMotor(0, 0, motor1_pwmA, motor1_pwmB);
  }
  
  motor1_prev_error = error;
}

void pidControlMotor2() {
  float error = motor2_target_speed - motor2_current_speed;
  motor2_integral += error * (CONTROL_PERIOD / 1000.0);
  float derivative = (error - motor2_prev_error) / (CONTROL_PERIOD / 1000.0);
  
  // Anti-windup for integral term
  if (motor2_integral > 50) motor2_integral = 50;
  if (motor2_integral < -50) motor2_integral = -50;
  
  float output = kp * error + ki * motor2_integral + kd * derivative;
  
  // Better output scaling - limit to reasonable range
  output = constrain(output, -200, 200);
  
  // Convert to PWM and direction
  int pwm_value = constrain(abs(output), 30, 255); // Minimum 30 for motor movement
  int direction = (output >= 0) ? 1 : -1;
  
  // Only apply PWM if target speed is not zero
  if (abs(motor2_target_speed) > 1.0) {
    setMotor(direction, pwm_value, motor2_pwmA, motor2_pwmB);
  } else {
    setMotor(0, 0, motor2_pwmA, motor2_pwmB);
  }
  
  motor2_prev_error = error;
}

void sendFeedback() {
  if (motor1_target_speed != 0 || motor2_target_speed != 0) {
    Serial.print("M1: Target=" + String(motor1_target_speed/COUNTS_PER_REV, 2) + "rev/s");
    Serial.print(" Actual=" + String(motor1_current_speed/COUNTS_PER_REV, 2) + "rev/s");
    Serial.print(" | M2: Target=" + String(motor2_target_speed/COUNTS_PER_REV, 2) + "rev/s");
    Serial.println(" Actual=" + String(motor2_current_speed/COUNTS_PER_REV, 2) + "rev/s");
  }
}

void stopMotors() {
  setMotor(0, 0, motor1_pwmA, motor1_pwmB);
  setMotor(0, 0, motor2_pwmA, motor2_pwmB);
  
  // Reset PID states
  motor1_integral = 0;
  motor2_integral = 0;
  motor1_prev_error = 0;
  motor2_prev_error = 0;
}



void setMotor(int dir,int pwmVal,int pin1,int pin2)
{
  if(dir == 1)
  {
    ledcWrite(pin1,pwmVal);
    ledcWrite(pin2,0);
  }
  else if(dir == -1)
  {
    ledcWrite(pin1,0);
    ledcWrite(pin2,pwmVal);
  }
  else
  {
    ledcWrite(pin1,0);
    ledcWrite(pin2,0);
  }
}

void debugEncoders() {
  Serial.println("=== ENCODER DEBUGGING ===");
  Serial.println("Starting 10-second encoder monitoring...");
  resetEncoders();
  
  unsigned long start_time = millis();
  long last_count1 = 0, last_count2 = 0;
  
  while (millis() - start_time < 10000) {
    if (encoder1_count != last_count1 || encoder2_count != last_count2) {
      Serial.println("Enc1: " + String(encoder1_count) + " | Enc2: " + String(encoder2_count) + 
                    " | Speed1: " + String(motor1_current_speed, 2) + " | Speed2: " + String(motor2_current_speed, 2));
      last_count1 = encoder1_count;
      last_count2 = encoder2_count;
    }
    
    // Continue speed calculations
    calculateSpeeds();
    delay(100);
  }
  
  Serial.println("=== DEBUG COMPLETE ===");
  Serial.println("Final counts - Enc1: " + String(encoder1_count) + " | Enc2: " + String(encoder2_count));
  Serial.println("Manually spin the motors and watch for count changes.");
}

void testMotor1Direction() {
  Serial.println("=== MOTOR 1 DIRECTION TEST ===");
  resetEncoders();
  
  Serial.println("Running Motor 1 forward for 3 seconds...");
  setMotor(1, 150, motor1_pwmA, motor1_pwmB);
  delay(3000);
  setMotor(0, 0, motor1_pwmA, motor1_pwmB);
  
  long forward_count = encoder1_count;
  Serial.println("Forward count: " + String(forward_count));
  
  delay(1000);
  
  Serial.println("Running Motor 1 backward for 3 seconds...");
  setMotor(-1, 150, motor1_pwmA, motor1_pwmB);
  delay(3000);
  setMotor(0, 0, motor1_pwmA, motor1_pwmB);
  
  long backward_count = encoder1_count;
  Serial.println("Backward count: " + String(backward_count));
  Serial.println("Net change: " + String(backward_count - forward_count));
  
  if (forward_count > 0 && backward_count < forward_count) {
    Serial.println("✓ Motor 1 encoder direction is CORRECT");
  } else {
    Serial.println("✗ Motor 1 encoder direction may be INVERTED");
  }
  Serial.println("=========================");
}

void testMotor2Direction() {
  Serial.println("=== MOTOR 2 DIRECTION TEST ===");
  resetEncoders();
  
  Serial.println("Running Motor 2 forward for 3 seconds...");
  setMotor(1, 150, motor2_pwmA, motor2_pwmB);
  delay(3000);
  setMotor(0, 0, motor2_pwmA, motor2_pwmB);
  
  long forward_count = encoder2_count;
  Serial.println("Forward count: " + String(forward_count));
  
  delay(1000);
  
  Serial.println("Running Motor 2 backward for 3 seconds...");
  setMotor(-1, 150, motor2_pwmA, motor2_pwmB);
  delay(3000);
  setMotor(0, 0, motor2_pwmA, motor2_pwmB);
  
  long backward_count = encoder2_count;
  Serial.println("Backward count: " + String(backward_count));
  Serial.println("Net change: " + String(backward_count - forward_count));
  
  if (forward_count > 0 && backward_count < forward_count) {
    Serial.println("✓ Motor 2 encoder direction is CORRECT");
  } else {
    Serial.println("✗ Motor 2 encoder direction may be INVERTED");
  }
  Serial.println("=========================");
}

void enterROSMode() {
  ros_mode = true;
  continuous_stream = false; // Start with streaming disabled
  last_ros_command_time = millis();
  stopAllMotors();
  
  Serial.println("=== ENTERING ROS MODE ===");
  Serial.println("ROS Commands:");
  Serial.println("  CMD_VEL <motor1_speed> <motor2_speed> - Set motor speeds (rev/s)");
  Serial.println("  START_STREAM - Enable continuous status streaming (100ms)");
  Serial.println("  STOP_STREAM - Disable continuous streaming");
  Serial.println("  GET_STATUS - Request single status update");
  Serial.println("  STOP - Stop all motors");
  Serial.println("  EXIT_ROS - Return to menu mode");
  Serial.println("Timeout: 2 seconds (motors stop if no command received)");
  Serial.println("ROS_MODE_READY");
}

void sendROSStatus() {
  // Format: "STATUS <motor1_target> <motor1_actual> <motor2_target> <motor2_actual> <enc1> <enc2>"
  Serial.print("STATUS ");
  Serial.print(motor1_target_speed / COUNTS_PER_REV, 3);
  Serial.print(" ");
  Serial.print(motor1_current_speed / COUNTS_PER_REV, 3);
  Serial.print(" ");
  Serial.print(motor2_target_speed / COUNTS_PER_REV, 3);
  Serial.print(" ");
  Serial.print(motor2_current_speed / COUNTS_PER_REV, 3);
  Serial.print(" ");
  Serial.print(encoder1_count);
  Serial.print(" ");
  Serial.println(encoder2_count);
}