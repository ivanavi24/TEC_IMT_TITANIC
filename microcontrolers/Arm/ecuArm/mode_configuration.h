#define MOD_ON                             1
#define MOD_OFF                            0

/*MODE FUNCTIONALITY HERE*/

#define NORMAL_FUNCTION                    1
#define CALIBRATE_MOTORS_POS               2
#define DISPLAY_MOTORS_VEL                 3
#define CALIBRATE_MOTORS_VEL               4
#define REACH_GIVEN_POSITION_SERIAL        5   



#define MOTOR1                  1
#define MOTOR2                  2
#define MOTOR3                  3



#define CURRENT_SELECTED_MOTOR      MOTOR2


/*SELECT MODE FUNCTIONALITY*/
#define CURRENT_OPERATION_MODE  CALIBRATE_MOTORS_VEL




/*Z axis crane configuration movement*/
#define CRANE_Z_AXIS_FREE                       1
#define CRANE_Z_AXIS_INTERMITENT                2
#define CRANE_Z_AXIS_LOCKED                     3

#define CRANE_Z_AXIS_BEHAVIOR                   CRANE_Z_AXIS_LOCKED

#if (CRANE_Z_AXIS_BEHAVIOR==CRANE_Z_AXIS_INTERMITENT)
/*Step in terms of revolutions of the step motor */
#define Z_AXIS_INTERMITENT_STEPS_LARGE                 2
#define Z_AXIS_INTERMITENT_STEPS_MIDDLE                1
#define Z_AXIS_INTERMITENT_STEPS_LOW                  0.5

#define Z_AXIS_STOP_STEPS                              Z_AXIS_INTERMITENT_STEPS_LARGE
#endif

#define DC_MOTOR_2ENCODERS                            MOD_OFF


#define SERIAL_PRINT_CONTROL_ESP32                      MOD_ON
