/* Declarations and Constants */
#include <RPLidar.h>
#include "Adafruit_VL53L0X.h"
#include <Arduino.h>

String packet;
String responseString;
double DIFFERENCE = 0; // value to increment numerical data by before responding
bool DEBUG = true; // If not debugging, set this to false to suppress debug messages

// Hardware Serial Definitions
char FRAMESTART = '[';
char FRAMEEND = ']';
int MAX_PACKET_LENGTH = 143; // equivalent to 16 8-byte commands of format "xx:#####", with 15 delimiting commas between them
int TIMEOUT = 250; // Hardware Serial timeout in milliseconds
int BAUDRATE = 9600;

// RPLidar Library and Definitions

#define RPLIDAR_MOTOR 7 // Control RPLIDAR motor PWM pin (MOTOCTRL).

RPLidar lidar;

// Store distance values for each direction segment
const int NUM_DIRECTIONS = 32; // 32 directions, each covering 11.25-degree segments
const int MAX_DISTANCES = 10; // Store only 10 distances per direction to save memory
float distanceMap[NUM_DIRECTIONS][MAX_DISTANCES]; // Array to store distances
int distanceCount[NUM_DIRECTIONS]; // Counter for each direction

// Encoder pins
#define ENCODER_A 2 //r
#define ENCODER_B 17 //r
#define ENCODER_C 3 //l
#define ENCODER_D 59 //l

// Motor pins
int enA = 10; //r
int in1 = 11;  //r
int in2 = 12; //r
int enB = 13; //l
int in3 = 4; //l
int in4 = 5; //l

// variables to store the number of encoder pulses for each motor
volatile long motCount_r = 0;
volatile long motCount_l = 0;
long count1inch = 141; // PPR = 1200, 1200(PPR) / 8.5 (inches per circumference) = 141 pulses per inch
long count1deg = 8; // PPR = 1200, 1200(PPR) / 8.5 (inches per circumference) * 6.75*pi (inches per self circle circumference) / 360 (degrees per revolution) = 8.32 pulses per 1 degree

// set up tof sensors

#define LOX_ADDRESS_BASE 0x30 // addresses we will assign for the sensors
const int shutdownPins[] = {52, 50, 48, 46, 44}; // set the pins to shutdown
Adafruit_VL53L0X lox[5] = {Adafruit_VL53L0X(), Adafruit_VL53L0X(), Adafruit_VL53L0X(), Adafruit_VL53L0X(), Adafruit_VL53L0X()}; // objects for the VL53L0X sensors
VL53L0X_RangingMeasurementData_t measures[5]; // measurement holders
float distances[5];

// receive
String cmdStr;
float msg_val;
bool stopFlag = false;

/* Create a debug message */
void debugMessage(String msg) {
  if (DEBUG) {
    Serial.println(msg);
  }
}

/* Serial receive function */
String receiveSerial3() {
  String frontmatter = "";
  String msg = "";
  char front_char = 0;
  char msg_char = 0;
  unsigned long start_time = millis();
  


  if (Serial3.available()) {
    // debugMessage("Raw packet received: " + msg);
    while (millis() < start_time + TIMEOUT) {
      if (Serial3.available()) {
        front_char = Serial3.read();
        if (front_char == FRAMESTART) {
          msg += front_char;
          break;
        } else {
          frontmatter += front_char;
        }
      }
    }
    if (frontmatter.length() > 0) {
      debugMessage("Prior to FRAMESTART, received: " + frontmatter);
    }

    while (millis() < start_time + TIMEOUT) {
      if (Serial3.available()) {
        msg_char = Serial3.read();
        if (msg_char == FRAMESTART) {
          debugMessage("A new framestart character was received, dumping: " + msg);
          msg = "";
        }
        msg += msg_char;
        if (msg_char == FRAMEEND) {
          break;
        }
      }
    }

    if (msg.length() < 1) {
      debugMessage("Timed out without receiving any message.");
    } else if (msg_char != FRAMEEND) {
      debugMessage("Timed out while receiving a message.");
    } else {
      debugMessage("Depacketizing received message:" + msg);
      return depacketize(msg);
    }
  }
  return "";
}

/* Remove packet framing information */
String depacketize(String msg) {
  if (msg.length() > 1 && msg[0] == FRAMESTART) {
    if (msg[msg.length()-1] == FRAMEEND) {
      return msg.substring(1, msg.length()-1);
    }
  }
  debugMessage("Missing valid packet framing characters.");
  return "";
}

/* Add packet framing information */
String packetize(String msg) {
  return FRAMESTART + msg + FRAMEEND +'\n';
}
/* Handle the received packet */
String parsePacket(String pkt) {
  int cmdStartIndex = 0;
  String responseString = "";
  for (int ct = 0; ct < pkt.length(); ct++) {
    if (pkt[ct] == ',') {
      responseString += parseCmd(pkt.substring(cmdStartIndex, ct)) + ',';
      cmdStartIndex = ct + 1;
    }
  }
  responseString += parseCmd(pkt.substring(cmdStartIndex));
  debugMessage("Response String is: " + responseString);
  return responseString;
}

/* Handle the received commands */
String parseCmd(String cmdString) {
  debugMessage("Parsing command: " + cmdString);

  String cmdID = cmdString.substring(0,min(2, cmdString.length()));
  double data = 0;


  if (cmdString.length() >= 4) {
    data = cmdString.substring(3).toDouble();
  }

  if (cmdID == "ld") {
    bool led_state = digitalRead(LED_BUILTIN);
    digitalWrite(LED_BUILTIN, !led_state);
    return cmdID + ':' + (!led_state ? "True" : "False");
  }

  if (cmdID == "lr") {
    String lidarData = read_lidar();  // Now read_lidar returns a String
    return lidarData;
  }


  if (cmdID == "xx") {
    stopFlag = true;
    Stop();
    Serial.println("Motors stopped");
  } else if (cmdID.charAt(0) == 'w') {
    msg_val = data; 
    if (msg_val < 0) {
      Back(-msg_val);
      Serial.print("back:");
      Serial.println(-msg_val);
    } else {
      Forward(msg_val);
      Serial.print("forward:");
      Serial.println(msg_val);
    }
  } else if (cmdID.charAt(0) == 'r') {
    msg_val = data; 
    if (msg_val < 0) {
      Left(-msg_val);
      Serial.print("left:");
      Serial.println(-msg_val);
    } else {
      Right(msg_val);
      Serial.print("right:");
      Serial.println(msg_val);
    }
  }

  return cmdID + ':' + String(data + DIFFERENCE);
}

// Initialize each sensor with unique I2C address
void setID() {
  // Reset all sensors by setting all shutdown pins LOW
  for (int i = 0; i < 5; i++) {
    digitalWrite(shutdownPins[i], LOW);
  }
  delay(20);

  // Sequentially initialize each sensor with a unique address
  for (int i = 0; i < 5; i++) {
    // Bring only one sensor out of reset at a time
    digitalWrite(shutdownPins[i], HIGH);
    delay(20);

    // Initialize sensor with a unique I2C address
    if (!lox[i].begin(LOX_ADDRESS_BASE + i)) {
      Serial.print(F("Failed to boot VL53L0X sensor "));
      Serial.println(i + 1);
      while (1);
    }
  }
}

// Read and print data from all sensors
float* read_sensors() {
  // Serial.println("Start TOF");
  for (int i = 0; i < 5; i++) {
    lox[i].rangingTest(&measures[i], false);

    // // // Print to Serial for monitoring
    // Serial.print(F("Sensor"));
    // Serial.print(i + 1);
    // Serial.print(F(": "));

    // // Print to Serial3 for Bluetooth
    Serial3.print(F("Sensor"));
    Serial3.print(i + 1);
    Serial3.print(F(": "));

    if (measures[i].RangeStatus != 4) {
      // Serial.print(measures[i].RangeMilliMeter);
      Serial3.print(measures[i].RangeMilliMeter);
      distances[i] = measures[i].RangeMilliMeter;
    } else {
      distances[i] = 1000; // out of range
      // Serial.print(F("Out of range"));
      Serial3.print(F("Out of range"));
    }
    // Serial.print(F(" "));
    Serial3.print(F(" "));
  }

  // Serial.println();
  Serial3.println();
  return distances;

}



void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  // right motor
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  // left motor
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  pinMode(ENCODER_C, INPUT);
  pinMode(ENCODER_D, INPUT);
  
  // initialize hardware interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), EncoderEvent_r, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_C), EncoderEvent_l, CHANGE);

  Serial.begin(115200);
  Serial.println("Setup complete"); // 测试输出，确保 Serial 正常工作
  Serial.setTimeout(TIMEOUT);
  Serial1.begin(115200);
  Serial3.begin(115200);

  lidar.begin(Serial1);
  pinMode(RPLIDAR_MOTOR, OUTPUT);
  analogWrite(RPLIDAR_MOTOR, 255);
  delay(2000);

   // set up tof sensors
  // Wait until the serial port opens for native USB devices
  while (!Serial) { delay(1); }

  // Initialize shutdown pins as output and reset all sensors
  for (int i = 0; i < 5; i++) {
    pinMode(shutdownPins[i], OUTPUT);
    digitalWrite(shutdownPins[i], LOW);
  }

  Serial.println(F("Shutdown pins initialized and sensors reset..."));
  Serial.println(F("Starting..."));

  // Assign unique addresses to each sensor
  setID();
}
void loop() {
  
  packet = receiveSerial3();
  if (packet.length() > 0) {
    debugMessage("Received good packet: " + packet);
  }

  if (packet.length() >= 2 && packet.length() <= MAX_PACKET_LENGTH) {
    responseString = parsePacket(packet);
    Serial3.print(packetize(responseString));
    // Serial.print(packetize(responseString));
    debugMessage("");
  }
  // Serial.print("New_Reading");
  read_sensors();
  // Serial.println(distances);
  
}

String read_lidar(){
  Serial.print("Lidar_Running");
  unsigned long startTime = millis();
  unsigned long scanDuration = 2000; // Scan for 2000 milliseconds

  // Clear previous data
  clearDistanceMap();

  while (millis() - startTime < scanDuration) {
    if (IS_OK(lidar.waitPoint())) {
      float distance = lidar.getCurrentPoint().distance;
      float angle = lidar.getCurrentPoint().angle;
      byte quality = lidar.getCurrentPoint().quality;

      // Only consider distances with sufficient quality
      if (quality > 14 && distance > 0) {
        int direction = mapAngleToDirection(angle);
        if (direction != -1 && distanceCount[direction] < MAX_DISTANCES) {
          distanceMap[direction][distanceCount[direction]++] = distance;
        }
      }
    }
  }

  // After the scan, process and output the representative distances
  String hexOutput = "lr:" + outputHexDistances();
  return hexOutput;  // Use clear delimiters
}

// encoder event for the interrupt call
void EncoderEvent_r() {
  if (digitalRead(ENCODER_A) == HIGH) {
    if (digitalRead(ENCODER_B) == LOW) {
      motCount_r++;
    } else {
      motCount_r--;
    }
  } else {
    if (digitalRead(ENCODER_B) == LOW) {
      motCount_r--;
    } else {
      motCount_r++;
    }
  }
}

void EncoderEvent_l() {
  if (digitalRead(ENCODER_C) == HIGH) {
    if (digitalRead(ENCODER_D) == LOW) {
      motCount_l++;
    } else {
      motCount_l--;
    }
  } else {
    if (digitalRead(ENCODER_D) == LOW) {
      motCount_l--;
    } else {
      motCount_l++;
    }
  }
}

void Forward(float forInches)
{
  motCount_r = 0;    // reset encoder count before start command
  motCount_l = 0;

  while (!stopFlag && (motCount_l < (count1inch * forInches)) && motCount_r > (count1inch * -forInches)) {
    // Drive motor forward
    digitalWrite(in1, LOW); //R
    digitalWrite(in2, HIGH); //R
    digitalWrite(in3, LOW);//L
    digitalWrite(in4, HIGH);//L
    analogWrite(enA, 200);  // motor speed (PWM value between 0-255)
    analogWrite(enB, 220);  // motor speed (PWM value between 0-255)

    // If stopFlag is true, stop the motors immediately
    if (stopFlag) {
      Stop();
      return;
    }
  }
  // Stop motor when the target distance is reached or stopFlag is set
  Stop();

  delay(500);
}

void Back(float backInches)
{
  motCount_r = 0;    // reset encoder count before start command
  motCount_l = 0;
  while (!stopFlag && (motCount_l > (count1inch * -backInches)) && motCount_r < (count1inch * backInches)) {
    // Drive motor backward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enA, 200);  // motor speed (PWM value between 0-255)
    analogWrite(enB, 220);  // motor speed (PWM value between 0-255)

    // If stopFlag is true, stop the motors immediately
    if (stopFlag) {
      Stop();
      return;
    }
  }
  // Stop motor when the target distance is reached or stopFlag is set
  Stop();
  delay(500);
}

void Left(float leftDeg)
{
  motCount_r = 0;    // reset encoder count before start command
  motCount_l = 0;
  while (!stopFlag && (motCount_l > (count1deg * -leftDeg)) && motCount_r > (count1deg * -leftDeg)) {
    // Drive motor backward
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enA, 200);  // motor speed (PWM value between 0-255)
    analogWrite(enB, 220);  // motor speed (PWM value between 0-255)

    // If stopFlag is true, stop the motors immediately
    if (stopFlag) {
      Stop();
      return;
    }
  }
  // Stop motor when the target distance is reached or stopFlag is set
  Stop();

  delay(500);
}

void Right(float rightDeg)
{
  motCount_r = 0;    // reset encoder count before start command
  motCount_l = 0;
  while (!stopFlag && (motCount_l < (count1deg * rightDeg)) && motCount_r < (count1deg * rightDeg)) {
    // Drive motor backward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enA, 200);  // motor speed (PWM value between 0-255)
    analogWrite(enB, 220);  // motor speed (PWM value between 0-255)

    // If stopFlag is true, stop the motors immediately
    if (stopFlag) {
      Stop();
      return;
    }
  }
  // Stop motor when the target distance is reached or stopFlag is set
  Stop();

  delay(500);
}

void Stop()
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enB, 0);
  analogWrite(enA, 0);
  motCount_r = 0;    // reset encoder count after stopping
  motCount_l = 0;
  // Clear the stopFlag after stopping
  stopFlag = false;
}

// Map an angle to one of 32 11.25-degree segments
int mapAngleToDirection(float angle) {
  return (int)(angle / 11.25); // Returns a value from 0 to 31 representing the direction
}

// Output the representative distance for each direction as 4-digit hex values
String outputHexDistances() {
  String hexOutput = "";

  for (int i = 0; i < NUM_DIRECTIONS; i++) {
    float representativeDistance = selectRepresentativeDistance(distanceMap[i], distanceCount[i]);

    // Convert distance to mm and round it to an integer
    int distance_mm = static_cast<int>(representativeDistance);

    // Convert the distance to a 4-character hexadecimal string
    String distance_hex = String(distance_mm, HEX);
    while (distance_hex.length() < 4) {
      distance_hex = "0" + distance_hex;  // Pad with leading zeros if necessary
    }

    // Add to the output string
    hexOutput += distance_hex;

  }

  Serial.println("Hex output to Serial3: " + hexOutput);
  return hexOutput;

  // delay(1000); // Wait before the next scan
}

// Select a representative distance (median) for a direction
float selectRepresentativeDistance(float* distances, int count) {
  if (count == 0) return -1; // Return -1 if no distances in this direction

  // Sort distances to find the median
  for (int i = 0; i < count - 1; i++) {
    for (int j = 0; j < count - i - 1; j++) {
      if (distances[j] > distances[j + 1]) {
        // Swap distances[j] and distances[j + 1]
        float temp = distances[j];
        distances[j] = distances[j + 1];
        distances[j + 1] = temp;
      }
    }
  }

  // Return the median value
  return distances[count / 2];
}

// Clear the distance map for the next scan
void clearDistanceMap() {
  for (int i = 0; i < NUM_DIRECTIONS; i++) {
    distanceCount[i] = 0;
  }
}
