import processing.serial.*;

String myString = null;
Serial myPort;  // The serial port

float[] temps =  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

// The statements in the setup() function 
// execute once when the program begins
void setup() {
  size(400, 400);  // Size must be the first statement
  noStroke();
  frameRate(30);
  
  // Print a list of connected serial devices in the console
  printArray(Serial.list());
  // Depending on where your GridEYE falls on this list, you
  // may need to change Serial.list()[0] to a different number
  myPort = new Serial(this, Serial.list()[0], 115200);
  myPort.clear();
  // Throw out the first chunk in case we caught it in the 
  // middle of a frame
  myString = myPort.readStringUntil(13);
  myString = null;
  // change to HSB color mode, this will make it easier to color
  // code the temperature data
  colorMode(HSB, 360, 100, 100);
}

// The statements in draw() are executed until the 
// program is stopped. Each statement is executed in 
// sequence and after the last line is read, the first 
// line is executed again.
void draw() { 
  // When there is a sizeable amount of data on the serial port
  // read everything up to the first linefeed
  if (myPort.available() > 64) {
    myString = myPort.readStringUntil(13);

    // readStringUntil is a non-blocking function and will return null if it can't find the linefeed
    if (myString == null) {
      return;
    }
    
    // generate an array of strings that contains each of the comma-separated values
    String splitString[] = splitTokens(myString, ",");
    
    // for each of the 64 values, map the temperatures between 20C and 35C
    // to the blue through red portion of the color space
    for (int q = 0; q < 64; q++) {
      temps[q] = map(float(splitString[q]), 20, 35, 240, 360);
    }
  }

  // Prepare variables needed to draw our heatmap
  int x = 0;
  int y = 0;
  int i = 0;
  background(0); // Clear the screen with a black background

  // each Grid-EYE pixel will be represented by a 50px square:
  // because 50 x 8 = 400, we draw squares until our y location
  // is 400
  while (y < 400) {
    // Flip the row index: subtract the current row index from 7 (0-based)
    int flippedRow = 7 - (y / 50);

    // for each increment in the y direction, draw 8 boxes in the
    // x direction, creating a 64 pixel matrix
    while (x < 400) {
      // Calculate the index in the temps array, considering the flipped row
      i = flippedRow * 8 + (x / 50);

      // Set the color for the current temperature
      fill(temps[i], 100, 100);
      rect(x, y, 50, 50);
      x = x + 50;
    }

    y = y + 50;
    x = 0;
  }

  // Add a Gaussian blur to the canvas in order to create a rough
  // visual interpolation between pixels.
  //filter(BLUR, 10);
}
