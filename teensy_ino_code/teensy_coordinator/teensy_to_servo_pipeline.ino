#include <Servo.h>
#include "Constants.h"

// Use Serial8 for communication with the Raspberry Pi.
// Adjust the baud rate as needed.
void setup() {
  Serial.begin(115200);      // Debug serial monitor.
  Serial1.begin(115200);     // Communication with the Raspberry Pi.
  
  // Attach servos to the defined pins.
  leg1_servo1.attach(LEG1_SERVO1_PIN);
  leg1_servo2.attach(LEG1_SERVO2_PIN);
  leg1_servo3.attach(LEG1_SERVO3_PIN);

  // // Attach servos to tShe defined pins.
  // leg2_servo1.attach(LEG2_SERVO1_PIN);
  // leg2_servo2.attach(LEG2_SERVO2_PIN);
  // leg2_servo3.attach(LEG2_SERVO3_PIN);

  // // Attach servos to the defined pins.
  // leg3_servo1.attach(LEG3_SERVO1_PIN);
  // leg3_servo2.attach(LEG3_SERVO2_PIN);
  // leg3_servo3.attach(LEG3_SERVO3_PIN);

  // // Attach servos to the defined pins.
  // leg4_servo1.attach(LEG4_SERVO1_PIN);
  // leg4_servo2.attach(LEG4_SERVO2_PIN);
  // leg4_servo3.attach(LEG4_SERVO3_PIN);
}

void loop() {
  //Check if data is available on Serial1.
  if(Serial.available()){
    Serial.println("0");
  }
  if(Serial1.available()){
    Serial.println("1");
  }
  if(Serial2.available()){
    Serial.println("2");
  }
  if(Serial3.available() || Serial4.available() || Serial5.available()){
    Serial.println("3");
  }
  if(Serial6.available() || Serial7.available() || Serial8.available()){
    Serial.println("4");
  }
  if (Serial.available()) {
    // Read the incoming string until newline. Expecting a command like "90,45"
    String command = Serial.readStringUntil('\n');
    command.trim();  // Remove any leading/trailing whitespace.
    //float leg_angles[4][3];
    String remainingStr = command;

    float leg1_angles[3];
    if (command.length() > 0) {
      for(int i=0; i<3; i++){
        int commaIndex = command.indexOf(',');
        if (commaIndex > 0) {
          String angleStr = remainingStr.substring(0, commaIndex);
          remainingStr = remainingStr.substring(commaIndex + 1);
          float angle = angleStr.toFloat();
          leg1_angles[i] = angle;
          Serial.print("Servo: ");
          Serial.print(i+1);
          Serial.print(" : ");
          Serial.println(angle);
        }
        else {
          Serial.println("Invalid command format. Expected: <angle1>,<angle2>, ...");
        } 
      }
      // float leg2_angles[3];
      // for(int i=0; i<3; i++){
      //   int commaIndex = command.indexOf(',');
      //   if (commaIndex > 0) {
      //     String angleStr = remainingStr.substring(0, commaIndex);
      //     remainingStr = remainingStr.substring(commaIndex + 1);
      //     float angle = angleStr.toFloat();
      //     leg2_angles[i] = angle;
      //     Serial.print("Servo: ");
      //     Serial.print(i+1);
      //     Serial.print(" : ");
      //     Serial.println(angle);
      //   }
      //   else {
      //     Serial.println("Invalid command format. Expected: <angle1>,<angle2>, ...");
      //   } 
      // }
      // float leg3_angles[3];
      // for(int i=0; i<3; i++){
      //   int commaIndex = command.indexOf(',');
      //   if (commaIndex > 0) {
      //     String angleStr = remainingStr.substring(0, commaIndex);
      //     remainingStr = remainingStr.substring(commaIndex + 1);
      //     float angle = angleStr.toFloat();
      //     leg3_angles[i] = angle;
      //     Serial.print("Servo: ");
      //     Serial.print(i+1);
      //     Serial.print(" : ");
      //     Serial.println(angle);
      //   }
      //   else {
      //     Serial.println("Invalid command format. Expected: <angle1>,<angle2>, ...");
      //   } 
      // }
      // float leg4_angles[3];
      // for(int i=0; i<3; i++){
      //   int commaIndex = command.indexOf(',');
      //   if (commaIndex > 0) {
      //     String angleStr = remainingStr.substring(0, commaIndex);
      //     remainingStr = remainingStr.substring(commaIndex + 1);
      //     float angle = angleStr.toFloat();
      //     leg4_angles[i] = angle;
      //     Serial.print("Servo: ");
      //     Serial.print(i+1);
      //     Serial.print(" : ");
      //     Serial.println(angle);
      //   }
      //   else {
      //     Serial.println("Invalid command format. Expected: <angle1>,<angle2>, ...");
      //   } 
      // }
    }

    // LEG 1 servos.
    leg1_servo1.write(leg1_angles[0]);
    leg1_servo2.write(leg1_angles[1]);
    leg1_servo3.write(leg1_angles[2]);

    // // LEG 2 servos.
    // leg2_servo1.write(leg2_angles[0]);
    // leg2_servo2.write(leg2_angles[1]);
    // leg2_servo3.write(leg2_angles[2]);

    // // LEG 3 servos.
    // leg3_servo1.write(leg3_angles[0]);
    // leg3_servo2.write(leg3_angles[1]);
    // leg3_servo3.write(leg3_angles[2]);

    // // LEG 4 servos.
    // leg4_servo1.write(leg4_angles[0]);
    // leg4_servo2.write(leg4_angles[1]);
    // leg4_servo3.write(leg4_angles[2]);

  }
}
