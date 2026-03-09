#ifndef TASK_PRIORITIES_H
#define TASK_PRIORITIES_H

/**
  * @file task_priorities.h
 * @brief
*/

#include "freertos/FreeRTOS.h"

// Largest number = higher priority
#define TASK_PRIO_6           6
#define TASK_PRIO_5           5   
#define TASK_PRIO_4           4     
#define TASK_PRIO_3           3        
#define TASK_PRIO_MAX       (configMAX_PRIORITIES - 1)

#endif  // TASK_PRIORITIES_H
