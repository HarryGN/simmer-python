/* Declarations and Constants */
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
#include <RPLidar.h>
#define RPLIDAR_MOTOR 7 // Control RPLIDAR motor PWM pin (MOTOCTRL).

RPLidar lidar;

// Store distance values for each direction segment
const int NUM_DIRECTIONS = 8; // 8 directions, each covering 45-degree segments
const int MAX_DISTANCES = 100; // Maximum number of distances to store for each segment
float distanceMap[NUM_DIRECTIONS][MAX_DISTANCES]; // Array to store distances
int distanceCount[NUM_DIRECTIONS]; // Counter for each direction
const int targetAngles[8] = {0, 45, 90, 135, 180, 225, 270, 315};

// Encoder pins
#define ENCODER_A 2 //r
#define ENCODER_B 17 //r
#define ENCODER_C 3 //l
#define ENCODER_D 59 //l

// Motor pins
int enA = 16; //r
int in1 = 1;  //r
int in2 = 58; //r
int enB = 13; //l
int in3 = 4; //l
int in4 = 5; //l

// variables to store the number of encoder pulses for each motor
volatile long motCount_r = 0;
volatile long motCount_l = 0;
long count1inch = 94.118;
long count1deg = 5.544;

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
  return FRAMESTART + msg + FRAMEEND;
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

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(13, OUTPUT);

  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  pinMode(ENCODER_C, INPUT);
  pinMode(ENCODER_D, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), EncoderEvent_r, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_C), EncoderEvent_l, CHANGE);

  Serial.begin(BAUDRATE);
  Serial.setTimeout(TIMEOUT);
  Serial1.begin(115200);
  Serial3.begin(115200);

  lidar.begin(Serial1);
  pinMode(RPLIDAR_MOTOR, OUTPUT);
  analogWrite(RPLIDAR_MOTOR, 255);
  delay(2000);
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

  unsigned long startTime = millis();
  unsigned long scanDuration = 1000; // Scan for 2000 milliseconds

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
  outputHexDistances();

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

/* Motor control functions */
void Forward(float forInches) {
  motCount_r = 0;
  motCount_l = 0;

  while (!stopFlag && (motCount_r < (count1inch * forInches)) && motCount_l > (count1inch * -forInches)) {
    digitalWrite(in1, LOW); //R
    digitalWrite(in2, HIGH); //R
    digitalWrite(in3, LOW); //L
    digitalWrite(in4, HIGH); //L
    analogWrite(enA, 235);
    analogWrite(enB, 150);
    Serial.print("motCount_r: ");
    Serial.println(motCount_r);
    Serial.print("motCount_l: ");
    Serial.println(motCount_l);


    if (stopFlag) {
      Stop();
      return;
    }
  }
  // digitalWrite(in3, LOW);
  // digitalWrite(in4, HIGH);
  // analogWrite(enA, 155);
  // delay(1000);
  Stop();
  delay(500);
}

void Back(float backInches) {
  motCount_r = 0;
  motCount_l = 0;
  while (!stopFlag && (motCount_r > (count1inch * -backInches)) && motCount_l < (count1inch * backInches)) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enA, 200);
    analogWrite(enB, 150);
    Serial.print("motCount_r: ");
    Serial.println(motCount_r);
    Serial.print("motCount_l: ");
    Serial.println(motCount_l);

    if (stopFlag) {
      Stop();
      return;
    }
  }
  Stop();
  delay(500);
}

void Left(float leftDeg) {
  motCount_r = 0;
  motCount_l = 0;
  while (!stopFlag && (motCount_r > (count1deg * -leftDeg)) && motCount_l < (count1deg * leftDeg)) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enA, 200);
    analogWrite(enB, 180);
    Serial.print("motCount_r: ");
    Serial.println(motCount_r);
    Serial.print("motCount_l: ");
    Serial.println(motCount_l);


    if (stopFlag) {
      Stop();
      return;
    }
  }
  Stop();
  delay(500);
}

void Right(float rightDeg) {
  motCount_r = 0;
  motCount_l = 0;
  while (!stopFlag && (motCount_r < (count1deg * rightDeg)) && motCount_l > (count1deg * -rightDeg)) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enA, 200);
    analogWrite(enB, 180);
    Serial.print("motCount_r: ");
    Serial.println(motCount_r);
    Serial.print("motCount_l: ");
    Serial.println(motCount_l);

    if (stopFlag) {
      Stop();
      return;
    }
  }
  Stop();
  delay(500);
}

void Stop() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enB, 0);
  analogWrite(enA, 0);
  motCount_r = 0;
  motCount_l = 0;
  stopFlag = false;
}

// Map an angle to one of eight 45-degree segments
int mapAngleToDirection(float angle) {
  if ((angle >= 337.5 && angle <= 360) || (angle >= 0 && angle < 22.5)) return 0;
  else if (angle >= 22.5 && angle < 67.5) return 1;
  else if (angle >= 67.5 && angle < 112.5) return 2;
  else if (angle >= 112.5 && angle < 157.5) return 3;
  else if (angle >= 157.5 && angle < 202.5) return 4;
  else if (angle >= 202.5 && angle < 247.5) return 5;
  else if (angle >= 247.5 && angle < 292.5) return 6;
  else if (angle >= 292.5 && angle < 337.5) return 7;
  return -1; // Invalid angle
}

// Output the representative distance for each direction as 4-digit hex values
void outputHexDistances() {
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

    // For debugging: print the distance and corresponding angle
    Serial.print("Angle: ");
    Serial.print(targetAngles[i]);
    Serial.print(" Distance: ");
    Serial.print(representativeDistance);
    Serial.print(" mm (Hex: ");
    Serial.print(distance_hex);
    Serial.println(")");
  }

  // Send the data over Serial3
  Serial3.println(hexOutput);
  Serial.println("Hex output to Serial3: " + hexOutput);

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
