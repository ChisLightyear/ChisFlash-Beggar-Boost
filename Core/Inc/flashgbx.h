#ifndef FLASHGBX_H
#define FLASHGBX_H

#include <stdbool.h>
#include <stdint.h>

void flashgbx_reset(void);
bool flashgbx_cancelled(void);
bool flashgbx_handle(void);

#endif
