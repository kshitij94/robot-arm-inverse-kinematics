

#include <Servo.h> // Include the Servo library
#include <SoftwareSerial.h>
int pos = 0; // variable to store the servo position
const byte numChars = 32;
char receivedChars[numChars];
int lastInput;
boolean newData = false;

// Declare a Servo object
Servo joint2Servo;
Servo joint5Servo;

void setup()
{
  Serial.begin(9600);
  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for Native USB only
  }

  // Attach the servo to a pin (e.g., digital pin 9).
  // You might need to change this pin number based on your wiring.
  joint2Servo.attach(9);
  joint5Servo.attach(10); // 0 - 180
}

void loop()
{
  recvWithStartEndMarkers();
  processNewData();
}

void recvWithStartEndMarkers()
{
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false)
  {
    rc = Serial.read();

    if (recvInProgress == true)
    {
      if (rc != endMarker)
      {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars)
        {
          ndx = numChars - 1; // Prevent buffer overflow
        }
      }
      else
      {
        receivedChars[ndx] = '\0'; // Terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }
    else if (rc == startMarker)
    {
      recvInProgress = true;
    }
  }
}

void processNewData2(int target)
{
  for (pos = 0; pos <= target; pos += 1)
  { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    joint2Servo.write(pos); // tell servo to go to position in variable 'pos'
    delay(15);              // waits 15 ms for the servo to reach the position
  }
}
void processNewData()
{
  if (newData == true)
  {
    newData = false;
    Serial.println();
    Serial.println("**");
    Serial.print("Received data from processor:");
    Serial.print(receivedChars);
    int degree = 0;
    int jointid = 0;
    char *commaPtr = strchr(receivedChars, ',');
    *commaPtr = '\0';
    degree = atoi(receivedChars);
    char *secondPartPtr = commaPtr + 1;
    char *secondCommaPtr = strchr(secondPartPtr, ',');
    *secondCommaPtr = '\0';
    jointid = atoi(secondPartPtr);
    Serial.println();
    Serial.print("Instructed to set joint");
    Serial.print(jointid);
    Serial.print(" to ");
    Serial.print(degree);
    Serial.println();
    if (jointid == 2)
    {
      joint2Servo.write(degree);
    }
    else
    {
      joint5Servo.write(degree);
    }
    delay(100);
    Serial.println("**");
  }
}