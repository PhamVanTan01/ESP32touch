#pragma once

#include <stdint.h>

/** Alarm severity per ISA-18.2 (Phase 3). */
typedef enum
{
    ALARM_SEVERITY_INFO = 0,
    ALARM_SEVERITY_WARNING,
    ALARM_SEVERITY_CRITICAL,
    ALARM_SEVERITY_COUNT
} alarm_severity_t;

/** Maximum number of active alarms stored (MISRA: bounded). */
#define ALARM_MAX_ACTIVE  32U
