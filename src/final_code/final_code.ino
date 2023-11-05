//Name: robot_firmware
//Author: Lentin Joseph
//ROS Arduino code publishing sensor data and subscribing to motor commands

#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <NewPing.h>
#include <SoftwareSerial.h>

// Define the software serial pins for Bluetooth communication
SoftwareSerial bluetooth(A0, A1);  // Define SoftwareSerial object for Bluetooth

////////////////////////////////////////////////////////////////////////////////
//Motor, Encoder and IR sensor pin definition
#define PULSES_PER_TURN  5 // Replace with the actual value for your encoder

int encoder_pinA = 3; //Motor A Left Motor
int encoder_pinB = 2; //Motor B Right Motor

volatile int pulses1 = 0;
volatile int pulses2 = 0;

#define ECHO_PIN 9
#define TRIGGER_PIN 8
#define MAX_DISTANCE 200 // Maximum distance (in cm) to measure
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // Create a NewPing object

//Motor A
int enableA = 6;
int MotorA1 = 13;
int MotorA2 = 12;

//Motor B
int enableB = 5;
int MotorB1 = 11;
int MotorB2 = 10;

/////////////////////////////////////////////////////////////////////////////////////

//ROS Node handle
ros::NodeHandle  nh;
//Left and right speed
int r_speed = 0, l_speed = 0;

//Direction flag for encoder
int left_direction = 1;
int right_direction = 1;

/////////////////////////////////////////////////////////////////////////////////////////////////////////

void left_speed_cb(const std_msgs::Int32& msg)  // cmd_vel callback function definition
{
  digitalWrite(LED_BUILTIN, HIGH - digitalRead(LED_BUILTIN)); // blink the led
  l_speed = msg.data;
}

void right_speed_cb(const std_msgs::Int32& msg)  // cmd_vel callback function definition
{
  digitalWrite(LED_BUILTIN, HIGH - digitalRead(LED_BUILTIN)); // blink the led
  r_speed = msg.data;
}

void reset_cb(const std_msgs::Bool& msg)
{
  l_speed = 0;
  r_speed = 0;

  pulses1 = 0;
  pulses2 = 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////
//Mapping function one range to another range

float mapFloat(float value, float fromLow, float fromHigh, float toLow, float toHigh) {
  return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

////////////////////////////////////////////////////////////////////////////////////////////////
//Publisher for Left and Right encoder

std_msgs::Int32 l_encoder_msg;
ros::Publisher l_enc_pub("left_ticks", &l_encoder_msg);

std_msgs::Int32 r_encoder_msg;
ros::Publisher r_enc_pub("right_ticks", &r_encoder_msg);

//Sharp distance publisher

std_msgs::Float32 sharp_msg;
ros::Publisher sharp_distance_pub("obstacle_distance", &sharp_msg);

//Subscribers for left and right speed

ros::Subscriber<std_msgs::Int32> left_speed_sub("set_left_speed", &left_speed_cb); // creation of subscriber object sub for recieving the cmd_vel
ros::Subscriber<std_msgs::Int32> right_speed_sub("set_right_speed", &right_speed_cb); // creation of subscriber object sub for recieving the cmd_vel
ros::Subscriber<std_msgs::Bool> reset_sub("reset", &reset_cb); // creation of subscriber object sub for recieving the cmd_vel

////////////////////////////////////////////////////////////////////////////////////////////////

// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;

// Loop frequency: 100 ms
const long interval = 50;

//////////////////////////////////////////////////////////////////////////////////////////////////////
//ISR for two encoders

void counter1() {
  pulses1 = pulses1 + left_direction;
}

void counter2() {
  pulses2 = pulses2 + right_direction;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setting encoder pins as interrupts

void setup_wheelencoder()
{
  pinMode(encoder_pinA, INPUT);
  attachInterrupt(digitalPinToInterrupt (encoder_pinA), counter1, RISING);
  pulses1 = 0;
  pinMode(encoder_pinB, INPUT);
  attachInterrupt(digitalPinToInterrupt (encoder_pinB), counter2, RISING);
  pulses2 = 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Read IR values and publish

void update_IR()
{
  unsigned int distance = sonar.ping_cm(); // Measure distance in centimeters

  if (distance == 0) {
    // If the distance is 0, it indicates an error or out-of-range measurement
    // Handle the error condition or discard the measurement
    // For example, you can publish a special value like -1 to indicate an error
    sharp_msg.data = -1;
  } else {
    sharp_msg.data = distance;
  }

  sharp_distance_pub.publish(&sharp_msg);

  //  Serial.print("Distance: ");
  //  Serial.print(distance);
  //  Serial.println(" cm");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void update_Motor()
{
  //If left speed is greater than zero
  if (l_speed >= 0)
  {
    //    Serial.print("l_speed >= 0");
    digitalWrite (MotorA1, LOW);
    digitalWrite (MotorA2, HIGH);
    analogWrite(enableA, abs(l_speed));
    left_direction = 1;
  }
  else
  {
    digitalWrite (MotorA1, HIGH);
    digitalWrite (MotorA2, LOW);
    analogWrite(enableA, abs(l_speed));
    left_direction = -1;
  }

  if (r_speed >= 0)
  {
    digitalWrite (MotorB1, HIGH);
    digitalWrite (MotorB2, LOW);
    analogWrite(enableB, abs(r_speed));
    right_direction = 1;
  }
  else
  {
    digitalWrite (MotorB1, LOW);
    digitalWrite (MotorB2, HIGH);
    analogWrite(enableB, abs(r_speed));
    right_direction = -1;
  }
}

void testBluetoothConnection()
{
  //  Serial.print("NodeHandle baud rate: ");
  //  Serial.println(nh.getHardware()->getBaud());
  // Check if there is data available on the Bluetooth connection
  if (bluetooth.available()) {
    char receivedData = bluetooth.read();
    delay(1000);
    if (receivedData == 't') {
      while (bluetooth.available()) bluetooth.read();  // Clear the buffer
      bluetooth.println("Connection test successful!");
    }
  }

  //Print the current baud rate
  bluetooth.print("NodeHandle baud rate: ");
  bluetooth.println(nh.getHardware()->getBaud());
}


void setup()
{
  // Initialize the software serial port for Bluetooth communication
  Serial.begin(9600);
  bluetooth.begin(9600);

  //  nh.getHardware()->setPort(&Serial1);de handle
  nh.getHardware()->setBaud(9600);

  pinMode (enableA, OUTPUT);
  pinMode (MotorA1, OUTPUT);
  pinMode (MotorA2, OUTPUT);

  pinMode (enableB, OUTPUT);
  pinMode (MotorB1, OUTPUT);
  pinMode (MotorB2, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);

  //Setup wheel encoders
  setup_wheelencoder();

  //Initialize ROS node
  nh.initNode();

  //Setup publisher
  nh.advertise(l_enc_pub);
  nh.advertise(r_enc_pub);
  nh.advertise(sharp_distance_pub);

  //Setup subscriber
  nh.subscribe(left_speed_sub);
  nh.subscribe(right_speed_sub);
  nh.subscribe(reset_sub);
}

void loop()
{
  testBluetoothConnection();
  
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;

    // Calculate wheel turns based on pulse count
    float left_turns = pulses1 / PULSES_PER_TURN;
    float right_turns = pulses2 / PULSES_PER_TURN;
    l_encoder_msg.data = pulses1;
    r_encoder_msg.data = pulses2;
    l_enc_pub.publish(&l_encoder_msg);
    r_enc_pub.publish(&r_encoder_msg);

    update_IR();
  }

  update_Motor();
  nh.spinOnce();

  delay(20);
}
