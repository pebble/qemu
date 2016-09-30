#include "qemu/typedefs.h"
#include "hw/arm/stm32.h"

typedef struct PebbleControl PebbleControl;

PebbleControl *pebble_control_create(CharDriverState *chr, Stm32Uart *uart);
PebbleControl *pebble_control_create_stm32f7xx(CharDriverState *chr, Stm32F7xxUart *uart);

void pebble_control_send_vibe_notification(PebbleControl *s, bool on);

