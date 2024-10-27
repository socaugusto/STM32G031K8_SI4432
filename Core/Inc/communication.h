#ifndef __COMMUNICATION_H__
#define __COMMUNICATION_H__

#ifdef __cplusplus
extern "C" {
#endif
#include "stdint.h"

uint8_t comm_init(void);
void comm_run(void);


#ifdef __cplusplus
}
#endif

#endif /* __COMMUNICATION_H__ */