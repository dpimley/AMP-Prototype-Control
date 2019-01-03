/*
 AMP prototype control

 Author - David Pimley
*/

#include <Servo.h>

#define pin_in_1 2  // digital out
#define pin_en_A 3  // PWM
#define pin_in_2 4  // digital out

#define pin_servo 5 // servo pwm pin

#define FORWARD 0   // forward flag
#define REVERSE 1   // reverse flag

#define R_GEAR_1_D 0.04 // 40 mm
#define R_GEAR_2_D 0.06 // 60 mm

#define WHEEL_DIAMETER 0.116 // 116 mm

#define F_GEAR_SERVO_D 0.025 // 25 mm
#define QUARTER_GEAR_CIRCUMFERENCE 0.019635 // 19.635 mm
#define STEERING_LEVER 0.030 // 30 mm

// MACROS

#define RAD_TO_DEG(x) (round((180 * x) / 3.14159))

/* 
   For Motor Control The Direction is Defined as:

   Setup:
   Red   --> OUT1 on L298N
   Black --> OUT2 on L298N

   Control: (Looking at top of motor)
   IN1 IN2 DIR
   H   L   CW
   L   H   CCW

   MAX_SPEED W/ CURRENT CONFIG --> 0.125 m/s
   MAX STEERING ANGLE          --> 
   
*/

#define RPM_POINTS 11
// Represents motor rpm for the respective pwm setting
const float rpm_values[]           = {0, 5.26, 14.34, 19.0, 23.1, 26.0, 27.4, 29.1, 29.3, 30.3, 32.0};
// The pwm setting for the respective motor rpm output
const unsigned char pwm_settings[] = {0,   50,    75,  100,  125,  150,  175,  200,  225,  250,  255};

// Instantiate the servo object for control
Servo servo;
   
void setup() {
  pinMode(pin_en_A, OUTPUT);
  pinMode(pin_in_1, OUTPUT);
  pinMode(pin_in_2, OUTPUT);
  servo.attach(pin_servo);
  Serial.begin(9600);
}

void loop() {
  move_steering(-0.174533);
  delay(5000);
  move_steering(0.610865);
  delay(5000);
}

/*
  Velocity is measured from the RPM of the motor.

  Example: Assuming 30 RPM on motor, the actual velocity
  of the vehicle will be:
  RPM @ Wheel Shaft --> (30 RPM) * (R_GEAR_1_D / R_GEAR_2_D)
  Meters Per Minute --> (RPM @ Wheel Shaft) * (pi * WHEEL_DIAMETER)
  Meters Per Second --> (Meters Per Minute) / 60 seconds per minute

  This function takes the inverse of this and calculates what motor
  RPM is necessary in order to get the desired velocity.

  Going forward is a postive motor speed
  Going backward is a negative motor speed
*/
void move_translation(char dir, float vel) {
  // Initialize local variables
  unsigned char pwm_setting = 0; // variable to hold the necessary pwm duty cycle setting
  
  // Return if velocity is zero as that indicates no movement
  if (vel == 0) {
   return;
  }

  // Get the pwm setting necessary for the specified velocity
  pwm_setting = get_pwm_value(vel);  

  // For forward movement, CW motor movement is needed
  if (FORWARD == dir && vel > 0) {
    set_vehicle_direction(FORWARD);
  }
  // For reverse movement, CCW motor movement is needed
  else if (REVERSE == dir && vel > 0) {
    set_vehicle_direction(REVERSE);
  }

  // Set the motor speed
  analogWrite(pin_en_A, pwm_setting);
}

/*
 This function moves the servo to the desired steering angle

 For consistency between the ROS framework ouput and the controls
 the angle will be measure in radians.

 The actual calculation will be used using the cosine rule.
 The movement needed will be based off of the steering angle
 given by ROS system.

 Going left is a positive angle in radians
 Going right is a negative angle in radians
 */
void move_steering(float angle) {
  // Initialize local variables
  float steering_distance = 0.0; // the distance the rack needs to move on the
                                 // steering mechanism to get the given steering angle
  float angle_delta = 0.0;       // the angle to be added / subtracted to get the full angle
  // Calculate steering distance needed
  steering_distance = sqrt((2 * (STEERING_LEVER * STEERING_LEVER)) - (2 * STEERING_LEVER * STEERING_LEVER * cos(angle)));

  // Determine distance necessary servo needs to turn
  // First check to see that the steering distance is not too large
  // If it is reverse the vehicle
  if (((F_GEAR_SERVO_D * 3.14159) / 4) < steering_distance) {
    servo.write(90);
    move_translation(REVERSE, 0.5);
    return;
  }

  // Otherwise calculate the necessary servo angle
  angle_delta = (steering_distance / QUARTER_GEAR_CIRCUMFERENCE) * 90;

  // An angle that is greater than 90 indicates a right turn on the servo
  // An angle that is less than 90 indicates a left turn on the servo
  if (angle < 0) {
    servo.write(90 + angle_delta);
  }
  else if (angle > 0) {
    servo.write(90 - angle_delta);
  }
  else {
    servo.write(90);
  }
}

/*
 The following function sets the motor configuration necessary
 in order to either move the vehicle forward or backwards

 Due to the gear reduction used movement is defined as follows:
 Forward --> CCW spin on the motor
 Reverse --> CW spin on the motor
 */
void set_vehicle_direction(char dir) {
  // From the table in the top comment forward movement indicates CCW movement
  if (FORWARD == dir) {
    digitalWrite(pin_in_1, LOW);
    digitalWrite(pin_in_2, HIGH);
  }
  // From the table in the top comment forward movement indicates CW movement
  else if (REVERSE == dir) {
    digitalWrite(pin_in_1, HIGH);
    digitalWrite(pin_in_2, LOW);
  }
  return;
}

/*
 This function calcuates the necessary duty cycle value
 necessary in order to get the desired velocity.

 The function uses linear interpolation from a set of hand
 calculated values in order to determine the appropriate value.

 TODO: Determine if a load factor needs to be used for the motor rpm
*/
int get_pwm_value(float vel) {
  // Initialize local variables
  float motor_rpm = 0.0;  // variable to hold the motor rpm necessary
  unsigned char pwm_setting = 0;  // variable to hold the calculated pwm setting

  // Calculate necessary motor rpm for the desired velocity
  motor_rpm = ((vel * 60) / (PI * WHEEL_DIAMETER)) / (R_GEAR_1_D / R_GEAR_2_D);

  // Perform linear interpolation on the given set of data to determine
  // the correct PWM duty cycle setting necessary to drive the motor.
  pwm_setting = perform_linear_interpolation(motor_rpm);

  return pwm_setting;
}

/*
 This function performs the linear interpolation needed to the PWM setting
 necessary to spin at the specified motor RPM
 */
int perform_linear_interpolation(float motor_rpm) {
  // Initialize local variables
  float rpm_diff_spec_low = 0.0;  // delta of specified motor rpm and low bound
  float rpm_diff_high_low = 0.0;  // delta of high rpm bound and low rpm bound
  float pwm_diff_high_low = 0;  // delta of high pwm bound and low pwm bound
  unsigned char resulting_pwm = 0;  // the resulting pwm setting
  unsigned char i = 0;  // iterator for while loop

  // Find data range that the specified motor_rpm lies in
  while (i < RPM_POINTS && (motor_rpm > rpm_values[i])) {
    i++;
  }

  // Check exit / edge cases

  // 1. RPM value not found or is too large for data set
  // therefore return 255 pwm duty cycle setting (max speed)
  if (RPM_POINTS == i) {
    return 255;
  }

  // 2. RPM value is zero or below (error case)
  // return 0
  if (0 == i) {
    return 0;
  }

  // 3. Normal case (i == 1 through i == 10)
  // Calculate rpm specified to low delta
  rpm_diff_spec_low = motor_rpm - rpm_values[i - 1];
  // Calculate rpm high to low bound delta
  rpm_diff_high_low = rpm_values[i] - rpm_values[i - 1];
  // Calculate pwm high to low bound delta
  pwm_diff_high_low = pwm_settings[i] - pwm_settings[i - 1];
  // Adding proportional delta to the low bound of the pwm settings
  resulting_pwm = ceil(pwm_settings[i - 1] + ((rpm_diff_spec_low / rpm_diff_high_low) * pwm_diff_high_low));
  return resulting_pwm;
}
