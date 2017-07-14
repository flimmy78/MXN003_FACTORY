#ifndef ATA_MODEM_2G_H__
#define ATA_MODEM_2G_H__

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif
	
bool mt2503_custom_common_hdlr(char *full_cmd_string);
	
#ifdef __cplusplus
}
#endif

#endif

