/* =========================================================================================
  Define controller constants */

#define THROTTLE_SCALER 0.5
#define kp 2                               // Altitude PID proportional constant 
#define ki 0                              // Altitude PID integral constant
#define kd 0.01                          // Altitude PID derivative constant
#define filtAlt 0.95                    // Altitude Estimation Filter constant (0-1)
#define LQRmult 0.45                   // Scaling factor for control law, varies between 0.5-1
#define LQR_P 0                     // LQR_P constant proportional prescaler
#define LQR_E 1.75                   // Integrator prescaler
#define INTEGRATOR_CLAMP 0.175      // Clamping term for integrator 
#define SLEW_LIMIT 10              // controller gimbal limit (higher = faster vehicle)
#define SLEW_FILTER 0.25           // Controller rate limiter (0-1), higher = slower/stabler
#define D_COMP 0.1             // dynamic damping coefficient. higher = more dynamic, lower = more damping
#define V_SPD 0.01              // Vertical speed reduction rate (depends on iteration time, faster code = lower value, typically 0.001-0.01)
#define DERIVATIVE_FILT 0.75   // Fintering term for low pass derivative filter

/* ==========================================================================================
  Define communication setup
*/
#ifndef RADIO_BAUDRATE
#define RADIO_BAUDRATE 57600    // Telemetry radio baudrates (use 57600)
#endif

#define LIDAR_BAUDRATE 115200   // LiDAR sensor UART speed. default is 115200

#ifndef USB_BAUDRATE
#define USB_BAUDRATE 250000     // USB Serial port baudtate. (N/A for USB mode)
#endif

#ifndef SERIAL_USB
#define SERIAL_USB Serial       // Serial port for USB communication (always Serial)
#endif

#ifndef RADIO_SERIAL
#define RADIO_SERIAL Serial4    // Serial port for radio communication
#endif

#ifndef LIDAR_SERIAL
#define LIDAR_SERIAL Serial1    // Serial port for LiDAR sensor
#endif

#define CMD_SERIAL Serial      // Listen port for waypoints
#define TELEMETRY1 Serial4     // wireless telemetry
#define TELEMETRY2 Serial      // wired telemetry

/* ==========================================================================================
  Define motor setup */

#define ESC1 6         // Pin for ESC1
#define ESC2 7         // Pin for ESC2
#define ESC3 5         // Pin for ESC3
#define ESC4 4         // Pin for ESC4
#define MAXVAL 1500    // highest speed controller may command esc
#define MINVAL 900     // lowest speed controller may command esc (other than 0)
#define IDLE_SPEED 20  // idle motor speed, range 12-20 ish, depending on esc calibration

#define E1CENTER 1         // Pin for ESC1
#define E2CENTER 1         // Pin for ESC2
#define E3CENTER 1         // Pin for ESC3
#define E4CENTER 1         // Pin for ESC4

/* Set the delay between iterations */
#define MAIN_DELAY 1

/* Define LQR gain matrix.
The rows define the inputs (U) to each Output value
The columns are the Full state (X) of the Vehicle
The linear math here is: 
                X' = AX 
                + K(R-X) 
                + K(R-Xint)

The Expansion of U is:
                        U[0] = total vehicle thrust
                        U[1] = moment about the X axis
                        U[2] = moment about the y axis
                        U[3] = moment about the z axis

So to "tune" and axis, say x, find the corresponding row, 
in this case the second row, and alter the values 
to choose how the moment about that axis is related to 
the respective columns of the vehicles state. (Curently the values
represent response to angular position about X, and angular speed about X)
*/

double K [4][6] = {{
    0, 0, 0, 0, 0, 0,
  }, {
    15, 0, 0, -100, 0, 0,
  }, {
    0, 15, 0, 0, -100, 0,
  }, {
    0, 0, 15, 0, -30, 0,
  },
};