// cc - by Johannes Heberlein 2014
// v 1.02
// thingiverse.com/joo   wiki.fablab-nuernberg.de
// units: mm; microseconds; radians
// origin: bottom left of drawing surface
// time library see http://playground.arduino.cc/Code/time
// RTC  library see http://playground.arduino.cc/Code/time
//               or http://www.pjrc.com/teensy/td_libs_DS1307RTC.html
// Change log:
// 1.01  Release by joo at https://github.com/9a/plotclock
// 1.02  Additional features implemented by Dave:
//       - added ability to calibrate servofaktor seperately for left and right servos
//       - added code to support DS1307, DS1337 and DS3231 real time clock chips
//       - see http://www.pjrc.com/teensy/td_libs_DS1307RTC.html for how to hook up the real time clock
//
// 1.03  Remix/mod by Kjetil Egeland
// 2.0  Adapted code base

// delete or mark the next line as comment if you don't need these
//#define CALIBRATION      // enable calibration mode
//#define REALTIMECLOCK    // enable real time clock

// When in calibration mode, adjust the following factor until the servos move exactly 90 degrees
#define SERVOFAKTORLEFT 610
#define SERVOFAKTORRIGHT 620

// Zero-position of left and right servo
// When in calibration mode, adjust the NULL-values so that the servo arms are at all times parallel
// either to the X or Y axis
//#define SERVOLEFTNULL 2250
//#define SERVORIGHTNULL 920
#define SERVOLEFTNULL 1650
#define SERVORIGHTNULL 500

#define SERVOPINLIFT  2
#define SERVOPINLEFT  3
#define SERVOPINRIGHT 4

// lift positions of lifting servo
//#define LIFT0 1080 // on drawing surface
//#define LIFT1 925  // between numbers
//#define LIFT2 725  // going towards sweeper
#define LIFT0 1120  // on drawing surface
#define LIFT1 825  // between numbers
#define LIFT2 605  // going towards sweeper

// speed of liftimg arm, higher is slower
//#define LIFTSPEED 1500
#define LIFTSPEED 3000

// length of arms
#define L1 35
#define L2 57.1
#define L3 13.2

// origin points of left and right servo
//#define O1X 22
//#define O1Y -25
//#define O2X 47
//#define O2Y -25
#define O1X 21
#define O1Y -25
#define O2X 48
#define O2Y -25

#define PARKX 77
#define PARKY 47
#define ERASEMAXX 60

#include <Servo.h>

int servoLift = 1500;

Servo servo1;  //
Servo servo2;  //
Servo servo3;  //

volatile double lastX = 75;
volatile double lastY = PARKY;

int last_min = 0;

void setup() {

    drawTo(PARKX, PARKY);
    lift(0);
    servo1.attach(SERVOPINLIFT);  //  lifting servo
    servo2.attach(SERVOPINLEFT);  //  left servo
    servo3.attach(SERVOPINRIGHT);  //  right servo
    delay(1000);

}

void loop()
{

#ifdef CALIBRATION

    // Servohorns will have 90° between movements, parallel to x and y axis
  drawTo(-3, 29.2);
  delay(500);
  drawTo(74.1, 28);
  delay(500);

#else

    int i = 0;
    if (last_min != minute()) {

        if (!servo1.attached()) servo1.attach(SERVOPINLIFT);
        if (!servo2.attached()) servo2.attach(SERVOPINLEFT);
        if (!servo3.attached()) servo3.attach(SERVOPINRIGHT);

        number(34, 25, 1, 0.9);

        number(48, 25, 2, 0.9);

        lift(2);
        drawTo(PARKX, PARKY);
        lift(1);

        servo1.detach();
        servo2.detach();
        servo3.detach();

        delay(5000);
    }

#endif

}

/**
 * Writes a number to the page.
 * @param bx    bottom left origin point x
 * @param by    bottom left origin point  y
 * @param num   number to write. "11" writes a ":" and "111" erases
 * @param scale size of the number (1 = 20mm high font)
 */
void number(float bx, float by, int num, float scale) {

    switch (num) {

        case 0:
            drawTo(bx + 12 * scale, by + 6 * scale);
            lift(0);
            bogenGZS(bx + 7 * scale, by + 10 * scale, 10 * scale, -0.8, 6.7, 0.5);
            lift(1);
            break;
        case 1:

            drawTo(bx + 3 * scale, by + 15 * scale);
            lift(0);
            drawTo(bx + 10 * scale, by + 20 * scale);
            drawTo(bx + 10 * scale, by + 0 * scale);
            lift(1);
            break;
        case 2:
            drawTo(bx + 2 * scale, by + 12 * scale);
            lift(0);
            bogenUZS(bx + 8 * scale, by + 14 * scale, 6 * scale, 3, -0.8, 1);
            drawTo(bx + 1 * scale, by + 0 * scale);
            drawTo(bx + 12 * scale, by + 0 * scale);
            lift(1);
            break;
        case 3:
            drawTo(bx + 2 * scale, by + 17 * scale);
            lift(0);
            bogenUZS(bx + 5 * scale, by + 15 * scale, 5 * scale, 3, -2, 1);
            bogenUZS(bx + 5 * scale, by + 5 * scale, 5 * scale, 1.57, -3, 1);
            lift(1);
            break;
        case 4:
            drawTo(bx + 10 * scale, by + 0 * scale);
            lift(0);
            drawTo(bx + 10 * scale, by + 20 * scale);
            drawTo(bx + 2 * scale, by + 6 * scale);
            drawTo(bx + 12 * scale, by + 6 * scale);
            lift(1);
            break;
        case 5:
            drawTo(bx + 2 * scale, by + 5 * scale);
            lift(0);
            bogenGZS(bx + 5 * scale, by + 6 * scale, 6 * scale, -2.5, 2, 1);
            drawTo(bx + 5 * scale, by + 20 * scale);
            drawTo(bx + 12 * scale, by + 20 * scale);
            lift(1);
            break;
        case 6:
            drawTo(bx + 2 * scale, by + 10 * scale);
            lift(0);
            bogenUZS(bx + 7 * scale, by + 6 * scale, 6 * scale, 2, -4.4, 1);
            drawTo(bx + 11 * scale, by + 20 * scale);
            lift(1);
            break;
        case 7:
            drawTo(bx + 2 * scale, by + 20 * scale);
            lift(0);
            drawTo(bx + 12 * scale, by + 20 * scale);
            drawTo(bx + 2 * scale, by + 0);
            lift(1);
            break;
        case 8:
            drawTo(bx + 5 * scale, by + 10 * scale);
            lift(0);
            bogenUZS(bx + 5 * scale, by + 15 * scale, 5 * scale, 4.7, -1.6, 1);
            bogenGZS(bx + 5 * scale, by + 5 * scale, 5 * scale, -4.7, 2, 1);
            lift(1);
            break;

        case 9:
            drawTo(bx + 9 * scale, by + 11 * scale);
            lift(0);
            bogenUZS(bx + 7 * scale, by + 15 * scale, 5 * scale, 4, -0.5, 1);
            drawTo(bx + 5 * scale, by + 0);
            lift(1);
            break;
        case 111: //Erase

            lift(0);
            drawTo(70, 46);
            drawTo(ERASEMAXX, 43);

            drawTo(ERASEMAXX, 49);
            drawTo(5, 49);
            drawTo(5, 45);
            drawTo(ERASEMAXX, 45);
            drawTo(ERASEMAXX, 40);

            drawTo(5, 40);
            drawTo(5, 35);
            drawTo(ERASEMAXX, 35);
            drawTo(ERASEMAXX, 30);

            drawTo(5, 30);
            drawTo(5, 25);
            drawTo(ERASEMAXX, 25);
            drawTo(ERASEMAXX, 20);

            drawTo(5, 20);
            drawTo(60, 44);

            drawTo(PARKX, PARKY);
            lift(2);

            break;

        case 11: //Draw ":"
            drawTo(bx + 5 * scale, by + 15 * scale);
            lift(0);
            bogenGZS(bx + 5 * scale, by + 15 * scale, 0.1 * scale, 1, -1, 1);
            lift(1);
            drawTo(bx + 5 * scale, by + 5 * scale);
            lift(0);
            bogenGZS(bx + 5 * scale, by + 5 * scale, 0.1 * scale, 1, -1, 1);
            lift(1);
            break;

    }
}


/**
 * Lifts the marker to a certain height
 * @param lift
 *          0: put marker down
 *          1: lift up a little bit
 *          2: lift up to remove cap
 */
void lift(char lift) {
    switch (lift) {
        // room to optimize  !

        case 0: //850

            if (servoLift >= LIFT0) {
                while (servoLift >= LIFT0)
                {
                    servoLift--;
                    servo1.writeMicroseconds(servoLift);
                    delayMicroseconds(LIFTSPEED);
                }
            }
            else {
                while (servoLift <= LIFT0) {
                    servoLift++;
                    servo1.writeMicroseconds(servoLift);
                    delayMicroseconds(LIFTSPEED);

                }

            }

            break;

        case 1: //150

            if (servoLift >= LIFT1) {
                while (servoLift >= LIFT1) {
                    servoLift--;
                    servo1.writeMicroseconds(servoLift);
                    delayMicroseconds(LIFTSPEED);

                }
            }
            else {
                while (servoLift <= LIFT1) {
                    servoLift++;
                    servo1.writeMicroseconds(servoLift);
                    delayMicroseconds(LIFTSPEED);
                }

            }

            break;

        case 2:

            if (servoLift >= LIFT2) {
                while (servoLift >= LIFT2) {
                    servoLift--;
                    servo1.writeMicroseconds(servoLift);
                    delayMicroseconds(LIFTSPEED);
                }
            }
            else {
                while (servoLift <= LIFT2) {
                    servoLift++;
                    servo1.writeMicroseconds(servoLift);
                    delayMicroseconds(LIFTSPEED);
                }
            }
            break;
    }
}

/**
 * Draws an art in the clockwise direction
 * @param bx        bottom x coordinate
 * @param by        bottom y coordinate
 * @param radius    the arc's radius
 * @param start     the start point in radians
 * @param end       the end point in radians
 * @param scale     scales the x axis to make the arc more oval. 1 is a circle, 0.5 is an oval
 */
void arc_clockwise(float bx, float by, float radius, int start, int end, float scale) {
    float incr = -0.05;
    float count = 0;

    do {
        drawTo(scale * radius * cos(start + count) + bx, radius * sin(start + count) + by);
        count += inc;
    }
    while ((start + count) > end);

}

/**
 * Draws an art in the counter-clockwise direction
 * @param bx        bottom x coordinate
 * @param by        bottom y coordinate
 * @param radius    the arc's radius
 * @param start     the start point in radians
 * @param end       the end point in radians
 * @param scale     scales the x axis to make the arc more oval. 1 is a circle, 0.5 is an oval
 */
void arc_counterclockwise(float bx, float by, float radius, int start, int end, float scale) {
    float inkr = 0.05;
    float count = 0;

    do {
        drawTo(scale * radius * cos(start + count) + bx,
               radius * sin(start + count) + by);
        count += inkr;
    }
    while ((start + count) <= end);
}

/**
 * Moves the marker from the current point to the new point
 * @param pX    x coordinate
 * @param pY    y coordinate
 */
void drawTo(double pX, double pY) {
    double dx, dy, c;
    int i;

    // dx dy of new point
    dx = pX - lastX;
    dy = pY - lastY;
    //path length in mm, times 4 equals 4 steps per mm
    c = floor(4 * sqrt(dx * dx + dy * dy));

    if (c < 1) c = 1;

    for (i = 0; i <= c; i++) {
        // draw line point by point
        set_XY(lastX + (i * dx / c), lastY + (i * dy / c));

    }

    lastX = pX;
    lastY = pY;
}


double return_angle(double a, double b, double c) {
    // cosine rule for angle between c and a
    return acos((a * a + c * c - b * b) / (2 * a * c));
}

/**
 * Sets the marker to an x,y point
 * @param Tx    x coordinate
 * @param Ty    z coordinate
 */
void set_XY(double Tx, double Ty)
{
    delay(1);
    double dx, dy, c, a1, a2, Hx, Hy;

    // calculate triangle between pen, servoLeft and arm joint
    // cartesian dx/dy
    dx = Tx - O1X;
    dy = Ty - O1Y;

    // polar lemgth (c) and angle (a1)
    c = sqrt(dx * dx + dy * dy); //
    a1 = atan2(dy, dx); //
    a2 = return_angle(L1, L2, c);

    servo2.writeMicroseconds(floor(((a2 + a1 - M_PI) * SERVOFAKTORLEFT) + SERVOLEFTNULL));

    // calculate joinr arm point for triangle of the right servo arm
    a2 = return_angle(L2, L1, c);
    Hx = Tx + L3 * cos((a1 - a2 + 0.621) + M_PI); //36,5°
    Hy = Ty + L3 * sin((a1 - a2 + 0.621) + M_PI);

    // calculate triangle between pen joint, servoRight and arm joint
    dx = Hx - O2X;
    dy = Hy - O2Y;

    c = sqrt(dx * dx + dy * dy);
    a1 = atan2(dy, dx);
    a2 = return_angle(L1, (L2 - L3), c);

    servo3.writeMicroseconds(floor(((a1 - a2) * SERVOFAKTORRIGHT) + SERVORIGHTNULL));

}