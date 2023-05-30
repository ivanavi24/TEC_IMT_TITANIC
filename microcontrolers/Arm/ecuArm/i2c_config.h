

#define START_COMMAND_WORLD_POS                       0x55 /* binary ->  0101 0101*/
#define END_COMMAND_WORLD_POS                         0xAA /* binary ->  1010 1010*/


#define START_COMMAND_LIMIT_SWITCHES                   0x55 /* binary ->  0101 0101*/
#define END_COMMAND__LIMIT_SWITCHES                         0xAA /* binary ->  1010 1010*/



#define BIT_SIZE_FORMAT                     0x0F
#define I2C_DEV_ADDR                        0x55

#define ARM_COMMAND_LENGTH                  0x08

#define ARM_WITH GRIPPER_COMMAND_LENGHT     0x6

#define STOP_TASKS_COMMAND_LENGTH           0x4

#define MAX_VALUE_CMD_1                     32.767
#define MIN_VALUE_CMD_1                     -32.767


#define EXTRA_MSG_OFFSET                    1   /*Extra byte is sended due to I2C Smbus Raspberry API */