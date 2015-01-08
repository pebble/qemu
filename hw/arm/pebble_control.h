#include "qemu/typedefs.h"
#include "hw/arm/stm32.h"

typedef struct PebbleControl PebbleControl;

PebbleControl  *pebble_control_create(CharDriverState *chr, Stm32Uart *uart);

void pebble_control_send_vibe_notification(PebbleControl *s, bool on);

