/*
 * Hello World!
 *
 * This is the Hello World! for Arduino. 
 * It shows how to send data to the computer
 */

int lineNumber = 0;
String inputString = "";
boolean stringComplete = false;

void setup()                    // run once, when the sketch starts
{
  Serial.begin(9600);           // set up Serial library at 9600 bps

  // reserve 200 bytes for the inputString:
  inputString.reserve(200);
  
}

void loop()                       // run over and over again
{
  //Serial.print("Number: ");  // prints hello with ending line break
  //Serial.println(lineNumber);

  // print the string when a newline arrives:
  if (stringComplete) {
    if (inputString == "PING\n") {
      lineNumber++;
      Serial.print("PONG: ");
      Serial.println(lineNumber);
    }
    // clear the string:
    inputString = "";
    stringComplete = false;
  }
  
  //delay(1000);
}

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
