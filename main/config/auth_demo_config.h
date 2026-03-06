#pragma once

#include "config/hmi_schema.h"

/*
 * Demo login credentials and role (single demo user).
 * Replace with NVS/schema or external auth in production.
 */
#define AUTH_DEMO_USERNAME        "admin"
#define AUTH_DEMO_PASSWORD        "admin"
#define AUTH_DEMO_ROLE            HMI_ROLE_MAINTENANCE_ADMIN

/* Default role when not logged in / after logout. */
#define AUTH_DEMO_DEFAULT_ROLE    HMI_ROLE_OPERATOR

/* Role level in schema treated as superuser (all permissions). */
#define AUTH_ROLE_LEVEL_SUPERUSER 40U

/* Login lockout after N failed attempts (ISO 25119). */
#define AUTH_MAX_FAILED_ATTEMPTS       5U
#define AUTH_LOCKOUT_DURATION_MINUTES 5U
