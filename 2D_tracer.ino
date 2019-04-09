/*
 * Project from
 * https://www.youtube.com/watch?v=opZ9RgmOIpc
*/
#include <Adafruit_MotorShield.h>

#define LINE_BUFFER_LENGTH 512

/*
   Configuration section
*/

// set the serial speed [2400, 9600, 19200, 38400, 57600, 115200, 250000, 500000, 1000000]
const int serialSpeed = 9600;

// Stepper config
char STEP = MICROSTEP;

// Motor steps to go 1 millimeter.
float StepsPerMillimeterX = 72.0;
float StepsPerMillimeterY = 72.0;

// Arcs are split into many line segments.  How long are the segments (mm)?
float MM_PER_SEGMENT = 0.1;

//fan pin (9 => bottom servo pin)
const int fan_pin = 9;

//laser pin (10 => top servo pin)
const int laser_pin = 10;

// Should be right for DVD steppers, but is not too important here
const int stepsPerRevolution = 48;

// do power the fan when laser on?
// Warning : the fan will not be turned down on finish.
boolean power_fan_when_lazer_on = true;

// Set to true to get debug output.
boolean verbose = false;

/*
   Setup section
*/

// Initialize steppers for X- and Y-axis for Adafruit motor shield v2
// Invert X & Y as we are moving supports, not the piece
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_StepperMotor *myStepperX = AFMS.getStepper(stepsPerRevolution, 1);
Adafruit_StepperMotor *myStepperY = AFMS.getStepper(stepsPerRevolution, 2);

/* Structures, global variables    */
struct point {
  float x;
  float y;
  float z;
};

// Current position of laser, in mm
struct point actuatorPos;

//  Drawing settings, should be OK
float StepInc = 1;
int StepDelay = 0;
int LineDelay = 0;

// Drawing robot limits, in mm
float Xmin = 0;
float Xmax = 37;
float Ymin = 0;
float Ymax = 37;

// laser power, 0 to 255 value, not a percent
int laser_min = 0;
int laser_max = 255;
// 20-30 engrave wood
// 10-15 engrave leather

// fan power
int fan_min = 0;
int fan_max = 255;

// start position, in steps
float Xpos = 0;
float Ypos = 0;

int laser_power = laser_min;
int fan_power = fan_max;

/*
   Initialisations
*/
void setup() {

  Serial.begin(serialSpeed);

  AFMS.begin();

  //Declaring motor pin as output and set maximum
  pinMode(fan_pin, OUTPUT);
  fan(fan_power);

  //Declaring laser pin as output and set minumum
  pinMode(laser_pin, OUTPUT);
  laser(laser_power);

  delay(100);

  // Decrease if necessary //!\\ not working
  myStepperX->setSpeed(255);
  myStepperY->setSpeed(255);

  //  Welcome message
  Serial.println("Starting DIY laser engraver v0.1");
  Serial.print("X range is from ");
  Serial.print(Xmin);
  Serial.print(" to ");
  Serial.print(Xmax);
  Serial.println(" mm.");
  Serial.print("Y range is from ");
  Serial.print(Ymin);
  Serial.print(" to ");
  Serial.print(Ymax);
  Serial.println(" mm.");
  Serial.print("Laser power ");
  Serial.print(laser_power);
  Serial.print(" / ");
  Serial.print(laser_max);
  Serial.println("");
}

/*
   Main loop
*/
void loop()
{

  delay(100);
  char line[LINE_BUFFER_LENGTH];
  char c;
  int lineIndex;
  bool lineIsComment, lineSemiColon;

  lineIndex = 0;
  lineSemiColon = false;
  lineIsComment = false;

  while (1) {     //!\\ Warning : the loop is closed

    // Serial reception - Mostly from Grbl, added semicolon support
    while ( Serial.available() > 0 ) {
      c = Serial.read();
      if (( c == '\n') || (c == '\r') ) {             // End of line reached
        if ( lineIndex > 0 ) {                        // Line is complete. Then execute!
          line[ lineIndex ] = '\0';                   // Terminate string
          if (verbose) {
            Serial.print( "Received : ");
            Serial.println( line );
          }
          boolean processed = processIncomingLine(line, lineIndex);
          if (!processed) {
            Serial.print( "Command not recognized : ");
            Serial.println( line );
          }
          lineIndex = 0;
        }
        else {
          // Empty or comment line. Skip block.
        }
        lineIsComment = false;
        lineSemiColon = false;
        Serial.println("ok");
      }
      else {
        if ( (lineIsComment) || (lineSemiColon) ) {   // Throw away all comment characters
          if ( c == ')' )  lineIsComment = false;     // End of comment. Resume line.
        }
        else {
          if ( c <= ' ' ) {                           // Throw away whitepace and control characters
          }
          else if ( c == '/' ) {                    // Block delete not supported. Ignore character.
          }
          else if ( c == '(' ) {                    // Enable comments flag and ignore all characters until ')' or EOL.
            lineIsComment = true;
          }
          else if ( c == ';' ) {
            lineSemiColon = true;
          }
          else if ( lineIndex >= LINE_BUFFER_LENGTH - 1 ) {
            Serial.println( "ERROR - lineBuffer overflow" );
            lineIsComment = false;
            lineSemiColon = false;
          }
          else if ( c >= 'a' && c <= 'z' ) {        // Upcase lowercase
            line[ lineIndex++ ] = c - 'a' + 'A';
          }
          else {
            line[ lineIndex++ ] = c;
          }
        }
      }
    }
  }
}

boolean processIncomingLine( char* line, int charNB ) {

  //  Needs to interpret
  //  G0 for quick move
  //  G1 for moving
  //  G2, G3 - Controlled Arc Move
  //  G4 P300 (wait 150ms)
  // http://marlinfw.org/meta/gcode/

  int currentIndex = 0;

  //parser opts (G and M)
  char optLetter = line[0];
  int optCode = -1;
  // hackfix for parsing the code
  if (optLetter == 'G')
    optCode = atoi(strchr( line, 'G' ) + 1);
  if (optLetter == 'M')
    optCode = atoi(strchr( line, 'M' ) + 1);

  boolean optX_present = strchr( line + currentIndex, 'X' ) != NULL;
  float optX = atof(strchr( line, 'X' ) + 1);                 //A coordinate on the X axis
  boolean optY_present = strchr( line + currentIndex, 'Y' ) != NULL;
  float optY = atof(strchr( line, 'Y' ) + 1);                 //A coordinate on the Y axis

  float optI = atof(strchr( line, 'I' ) + 1);                 //An offset from the current X position to use as the arc center
  float optJ = atof(strchr( line + currentIndex, 'J' ) + 1);  //An offset from the current Y position to use as the arc center
  float optR = atof(strchr( line + currentIndex, 'R' ) + 1);  //A radius from the current XY position to use as the arc center
  int optP = atoi(strchr( line + currentIndex, 'P' ) + 1);    //Amount of time to dwell in msec
  boolean optS_present = strchr( line + currentIndex, 'S' ) != NULL;
  int optS = atoi(strchr( line + currentIndex, 'S' ) + 1);    //Amount of time to dwell in sec, Spindle speed or laser power
  int optD = atoi(strchr( line + currentIndex, 'D' ) + 1);    //Detailed things : http://marlinfw.org/docs/gcode/M114.html

  if (false) // debug command line
  {
    Serial.print("Parsing: ");
    Serial.print(line);
    Serial.print(", chars: ");
    Serial.println(charNB);

    Serial.print("Letter : ");
    Serial.println(optLetter);
    Serial.print("Code : ");
    Serial.println(optCode);

    Serial.print("opt X : ");
    Serial.println(optX);
    Serial.print("opt Y : ");
    Serial.println(optY);
    Serial.print("opt I : ");
    Serial.println(optI);
    Serial.print("opt J : ");
    Serial.println(optJ);
    Serial.print("opt R : ");
    Serial.println(optR);
    Serial.print("opt P : ");
    Serial.println(optP);
    Serial.print("opt S : ");
    Serial.print("opt D : ");
    Serial.println(optS);
  }

  switch (optLetter) {                // Select command (G / M)
    case 'G':
      switch (optCode) {              // Select G command
        case 0:                       // G00 & G01 - Movement or fast movement. Same here
        case 1:
          if (optX_present || optY_present) // disable if X & Y not present
            G0(optX, optX_present, optY, optY_present);
          return true;
        case 2:                       // G2 - Controlled Arc Move clockwise
          G2(optX, optY, optI, optJ, optR, false);
          return true;
        case 3:                       // G3 - Controlled Arc Move counter-clockwise
          G2(optX, optY, optI, optJ, optR, true);
          return true;
        case 4:                       // G4 - Dwell (pause)
          G4(optP, optS);
          return true;
        case 28:                      // hack for homing
          G0(0, 0);
          return true;
        case 92:                      // G92 Xx Yy - Set Position
          G92(optX, optY);
          return true;
        default:
          return false;
      }
      break;
    case 'M':
      switch (optCode) {
        case 3:                       // M3 Spow - Spindle CW / Laser On (Constant Laser Power Mode) https://github.com/gnea/grbl/wiki/Grbl-v1.1-Laser-Mode
        case 4:                       // M4 - Dynamic Laser Power Mode
          if (!optS_present) optS = laser_max; //special case for S non present, set lazer to max
          M3(optS);
          return true;
        case 5:                       // M5 - Spindle / Laser Off
          M5();
          return true;
        case 18:                      // M18 - Disable steppers
        case 84:                      // M84 - Disable steppers
          M18();
          return true;
        case 106:                     // M106 - Set Fan Speed
          if (!optS_present) optS = fan_max; //special case for S non present, set fan to max
          M106(optS);
          return true;
        case 107:                     // M107 - Fan Off
          M107();
          return true;
        case 112:                     // M112 - Emergency Stop
          M112();
          return true;
        case 114:                     // M114 - Get Current Position
          M114();
          return true;
        case 300:                     // M300 - Hacky code for provided gcodes files
          if (optS == 30)             // laser on
            M3(laser_max);
          else if (optS == 50)        // laser off
            M3(laser_min);
          else if (optS == 255)       // laser off
            M3(0);
          return true;
        default:
          return false;
      }
    default:
      return false;
  }
}


/*********************************
   G-code functions
**********************************/
//G0, G1 - Linear Move
void G0(float x, float y) {
  G0(x, true, y, true);
}
void G0(float x, boolean moveX, float y, boolean moveY) {
  point newPos;
  newPos.x = 0.0;
  newPos.y = 0.0;
  if ( moveX && !moveY ) {
    newPos.x = x;
    newPos.y = actuatorPos.y;
  }
  else if ( !moveX && moveY ) {
    newPos.x = actuatorPos.x;
    newPos.y = y;
  }
  else {
    newPos.x = x;
    newPos.y = y;
    y = '\0';
  }
  drawLine(newPos.x, newPos.y);
  actuatorPos.x = newPos.x;
  actuatorPos.y = newPos.y;
}
// G2,G3 http://marlinfw.org/docs/gcode/G002-G003.html (offset or radius)
void G2(float endX, float endY, float xCenterOffset, float yCenterOffset, float radius, boolean counterClockwise) {
  //  example : G3 X3 Y3 I5 J5
  // -X : endX absolute position
  // -Y : endY absolute position
  // -I : centerX offset relative from position
  // -J : centerY offset relative from position

  //  example : G3 X3 Y3 R2
  // -X : endX
  // -Y : endY
  // -R : radius

  arc(actuatorPos.x + xCenterOffset, actuatorPos.y + yCenterOffset, endX, endY, radius, counterClockwise ? 1 : -1);
}

// G4 - Dwell
void G4(int msec, int sec) {
  if (verbose) {
    Serial.print("Wait for ");
    if (msec > 0) {
      Serial.print(msec);
      Serial.print(" millisecond");
    } else if (sec > 0) {
      Serial.print(sec);
      Serial.print(" second");
    } else {
      Serial.println(" until all moves in the planner are completed");
    }
  }
  if (msec > 0) {
    delay(msec);
  } else if (sec > 0) {
    delay(msec * 1000);
  } else {
    //all actions finished
  }
}
// G92 - Set Position (without moving)
void G92(int x, int y) {
  actuatorPos.x = x;
  actuatorPos.y = y;
  Xpos = (int)(x * StepsPerMillimeterX);
  Ypos = (int)(y * StepsPerMillimeterY);
  if (verbose) {
    Serial.print("Set Position to X");
    Serial.print(x);
    Serial.print(" Y");
    Serial.println(y);
  }
}

// M3 Laser On
void M3 (int power) {
  laser(power);
}
// M4 Laser On (Dynamic Laser Power Mode)
void M4 (int power) {
  laser(power);
  if (verbose) {
    Serial.print("Dynamic Laser Power Mode On, power : ");
    Serial.println(power);
  }
}
// M5 Laser Off
void M5 () {
  laser(0);
  if (verbose) {
    Serial.println("Laser Off");
  }
}
// M17 Enable Steppers
void M17 () {
  if (verbose) {
    Serial.println( "Enable Steppers" );
  }
}
// M18/M84 Disable steppers
void M18 () {
  if (verbose) {
    Serial.println( "Disable steppers" );
  }
  myStepperX->release();
  myStepperY->release();
}
// M106 - Set Fan Speed
void M106 (int power) {
  fan(power);
}
// M107 - Fan Off
void M107 () {
  fan(0);
  if (verbose) {
    Serial.println("Fan Off");
  }
}
// M112 Emergency Stop
void M112 () {
  if (verbose) {
    Serial.println("Emergency Stop");
  }
  laser(0);
  fan(0);
  myStepperX->release();
  myStepperY->release();
}
// M114 Get Current Position
void M114 () {
  Serial.print("X: ");
  Serial.print(actuatorPos.x);
  Serial.print(" Y: ");
  Serial.print(actuatorPos.y);
  Serial.println(" mm");
  Serial.print("X2: ");
  Serial.print(Xpos);
  Serial.print(" Y2: ");
  Serial.print(Ypos);
  Serial.println(" steps");
}



/*********************************
   Draw a line from (x0;y0) to (x1;y1).
   int (x1;y1) : Starting coordinates
   int (x2;y2) : Ending coordinates
 **********************************/
void drawLine(float x1, float y1) {

  if (verbose)
  {
    Serial.print("fx1, fy1: ");
    Serial.print(x1);
    Serial.print(",");
    Serial.print(y1);
    Serial.println("");
  }

  //  Bring instructions within limits
  if (x1 >= Xmax) {
    x1 = Xmax;
  }
  if (x1 <= Xmin) {
    x1 = Xmin;
  }
  if (y1 >= Ymax) {
    y1 = Ymax;
  }
  if (y1 <= Ymin) {
    y1 = Ymin;
  }

  if (verbose)
  {
    Serial.print("Xpos, Ypos: ");
    Serial.print(Xpos);
    Serial.print(",");
    Serial.print(Ypos);
    Serial.println("");
  }

  if (verbose)
  {
    Serial.print("x1, y1: ");
    Serial.print(x1);
    Serial.print(",");
    Serial.print(y1);
    Serial.println("");
  }

  //  Convert coordinates to steps
  x1 = floor(x1 * StepsPerMillimeterX);
  y1 = floor(y1 * StepsPerMillimeterY);
  // store current position
  float x0 = Xpos;
  float y0 = Ypos;

  //  Let's find out the change for the coordinates
  long dx = abs(x1 - x0);
  long dy = abs(y1 - y0);
  int sx = x0 < x1 ? StepInc : -StepInc;
  int sy = y0 < y1 ? StepInc : -StepInc;

  long i;
  long over = 0;

  if (dx > dy) {
    for (i = 0; i < dx; ++i) {
      myStepperX->onestep(sx, STEP);
      over += dy;
      if (over >= dx) {
        over -= dx;
        myStepperY->onestep(sy, STEP);
      }
      delay(StepDelay);
    }
  }
  else {
    for (i = 0; i < dy; ++i) {
      myStepperY->onestep(sy, STEP);
      over += dx;
      if (over >= dy) {
        over -= dy;
        myStepperX->onestep(sx, STEP);
      }
      delay(StepDelay);
    }
  }

  if (verbose)
  {
    Serial.print("dx, dy:");
    Serial.print(dx);
    Serial.print(",");
    Serial.print(dy);
    Serial.println("");
  }

  if (verbose)
  {
    Serial.print("Going to (");
    Serial.print(x0);
    Serial.print(",");
    Serial.print(y0);
    Serial.println(")");
  }

  //  Delay before any next lines are submitted
  delay(LineDelay);
  //  Update the positions
  Xpos = x1;
  Ypos = y1;
}

// from https://www.marginallyclever.com/2014/03/how-to-improve-the-2-axis-cnc-gcode-interpreter-to-understand-arcs/

// This method assumes the limits have already been checked.
// This method assumes the start and end radius match.
// This method assumes arcs are not >180 degrees (PI radians)
// centerX/centerY - center of circle (absolute, in mm)
// endX/endY - end position (absolute, in mm)
// radius - radius of arc (not provided if centerX/centerY are)
// dir - ARC_CW or ARC_CCW to control direction of arc

//              I              J              X           Y
void arc(float centerX, float centerY, float endX, float endY, float radius, float dir) {

  // get radius
  float relativeCenterX = actuatorPos.x - centerX;
  float relativeCenterY = actuatorPos.y - centerY;

  if (radius == 0) //if radius is not provided, calculate it with Pythagorean theorem!
    radius = sqrt(relativeCenterX * relativeCenterX + relativeCenterY * relativeCenterY);

  // find angle of arc (sweep)
  float angle1 = atan3(relativeCenterY, relativeCenterX);
  float angle2 = atan3(endY - centerY, endX - centerX);
  float theta = angle2 - angle1;

  if (dir > 0 && theta < 0)
    angle2 += 2 * PI;
  else if (dir < 0 && theta > 0)
    angle1 += 2 * PI;

  theta = angle2 - angle1;

  // get length of arc
  float len = abs(theta) * radius;

  int segmentIndex, segments = ceil( len / MM_PER_SEGMENT );

  if (verbose) {
    Serial.println("------------start arc------------");
    Serial.print("center = (");
    Serial.print(centerX);
    Serial.print(",");
    Serial.print(centerY);
    Serial.print("), start = (");
    Serial.print(actuatorPos.x);
    Serial.print(",");
    Serial.print(actuatorPos.y);
    Serial.print("), radius = ");
    Serial.print(radius);
    Serial.print(", theta = ");
    Serial.print(theta);
    Serial.print(", length = ");
    Serial.print(len);
    Serial.print(", segments = ");
    Serial.print(segments);
    Serial.println("");
  }

  float segmentX, segmentY, angle3, scale;

  for (segmentIndex = 0; segmentIndex < segments; ++segmentIndex) {
    // interpolate around the arc
    scale = ((float)segmentIndex) / ((float)segments);
    angle3 = ( theta * scale ) + angle1;

    // find the intermediate position
    segmentX = centerX + cos(angle3) * radius;
    segmentY = centerY + sin(angle3) * radius;

    if (verbose) {
      if (segmentX > Xmax || segmentY > Ymax || segmentX < Xmin || segmentY < Ymin) {
        Serial.println("------------TOO FAR------------");
        Serial.print("Going to (");
        Serial.print(segmentX);
        Serial.print(",");
        Serial.print(segmentY);
        Serial.println(")");
        return;
      } else {
        Serial.println("------------segment------------");
        Serial.print("Index : ");
        Serial.println(segmentIndex);
        Serial.print("Going to (");
        Serial.print(segmentX);
        Serial.print(",");
        Serial.print(segmentY);
        Serial.println(")");
      }
    }
    // trace segment
    G0(segmentX, segmentY);
  }
  if (verbose) {
    Serial.println("------------end arc------------");
    Serial.print("Going to (");
    Serial.print(endX);
    Serial.print(",");
    Serial.print(endY);
    Serial.println(")");
    return;
  }
  G0(endX, endY);
}

// Laser control
void laser(int power) {
  analogWrite(laser_pin, power);
  if (power > laser_max)
    power = laser_max;
  if (power_fan_when_lazer_on)
    fan(fan_max);
  laser_power = power;
  if (verbose) {
    Serial.print("Laser power : ");
    Serial.println(power);
  }
}
// Fan control
void fan(int power) {
  analogWrite(fan_pin, power);
  if (power > fan_max)
    power = fan_max;
  fan_power = power;
  if (verbose) {
    Serial.print("Fan power : ");
    Serial.println(power);
  }
}

// returns angle of dy/dx as a value from 0...2PI
float atan3(float dy, float dx) {
  float a = atan2(dy, dx);
  if (a < 0)
    a = (PI * 2.0) + a;
  return a;
}
