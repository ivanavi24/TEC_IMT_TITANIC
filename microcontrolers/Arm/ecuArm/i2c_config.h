
#define MAX_VALUE_CMD_1                     32.767
#define MIN_VALUE_CMD_1                     -32.767
#define BIT_LEN_POS_DATA                    16

#define START_COMMAND_WORLD_POS                       0x55 /* binary ->  0101 0101*/
#define END_COMMAND                       0xAA /* binary ->  1010 1010*/

#define START_COMMAND_LIMIT_SWITCHES                   0x54 /* binary ->  0101 0101*/

#define START_COMMAND_SAILING                0x53 /* binary ->  0101 0101*/

#define START_COMMAND_SCANING            0x51 /* binary ->  0101 0101*/



#define BIT_SIZE_FORMAT                     0x0F
#define I2C_DEV_ADDR                        0x55

#define ARM_COMMAND_LENGTH                  0x08

#define ARM_WITH GRIPPER_COMMAND_LENGHT     0x6

#define STOP_TASKS_COMMAND_LENGTH           0x4




#define EXTRA_MSG_OFFSET                    1   /*Extra byte is sended due to I2C Smbus Raspberry API */