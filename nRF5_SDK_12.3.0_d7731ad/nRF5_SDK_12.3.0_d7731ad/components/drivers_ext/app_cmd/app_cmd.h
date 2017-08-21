#include "stdint.h"
typedef struct 
{
	short  position;
	short  length;
	char   character[20];
} custom_cmdLine;

typedef enum
{
	CUSTOM_RSP_ERROR = -1,
	CUSTOM_RSP_OK = 0,
	CUSTOM_RSP_LATER
} custom_rsp_type_enum;

typedef struct
{
	char *commandString;
	custom_rsp_type_enum (*commandFunc)(custom_cmdLine *commandBuffer_p);
} custom_atcmd;

typedef enum
{
	CUSTOM_WRONG_MODE,
	CUSTOM_SET_OR_EXECUTE_MODE,
	CUSTOM_READ_MODE,
	CUSTOM_TEST_MODE,
	CUSTOM_ACTIVE_MODE
} custom_cmd_mode_enum;

typedef struct
{
	short  position;
	uint8_t   	 part;
	char     	 	 rcv_msg[20];
	char       	 *pars[20];
	uint16_t      rcv_length;
} cmd_data_struct;

typedef enum
{
    CM_Main,
    CM_Par1,
    CM_Par2,
    CM_Par3,
    CM_Par4,
    CM_Par5,
    CM_Par6,
    CM_Par7,
    CM_Par8,
    CMD_MAX_Pars
} Cmd_Pars;


