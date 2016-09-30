#include <stdint.h>
#include <stdbool.h>
#include <qemu/typedefs.h>
#include "stm32f2xx.h"
#include "stm32f4xx.h"
#include "stm32f7xx.h"

typedef enum {
    PBL_BUTTON_ID_NONE = -1,
    PBL_BUTTON_ID_BACK = 0,
    PBL_BUTTON_ID_UP = 1,
    PBL_BUTTON_ID_SELECT = 2,
    PBL_BUTTON_ID_DOWN = 3,
    PBL_NUM_BUTTONS = 4
} PblButtonID;

typedef struct {
    int gpio;
    int pin;
    bool active_high;
} PblButtonMap;

typedef struct {
  int dbgserial_uart_index;
  int pebble_control_uart_index;

  PblButtonMap button_map[PBL_NUM_BUTTONS];
  uint32_t gpio_idr_masks[STM32F4XX_GPIO_COUNT];

  // memory sizes
  uint32_t flash_size;
  uint32_t ram_size;
  // screen sizes
  uint32_t num_rows;
  uint32_t num_cols;
  uint32_t num_border_rows;
  uint32_t num_border_cols;
  bool row_major;
  bool row_inverted;
  bool col_inverted;
  bool round_mask;
} PblBoardConfig;

void pebble_32f412_init(MachineState *machine, const PblBoardConfig *board_config);
void pebble_32f439_init(MachineState *machine, const PblBoardConfig *board_config);
void pebble_32f7xx_init(MachineState *machine, const PblBoardConfig *board_config);

// This method used externally (by pebble_control) for setting the button state
void pebble_set_button_state(uint32_t button_state);
void pebble_set_qemu_settings(DeviceState *rtc_dev);
void pebble_connect_uarts(Stm32Uart *uart[], const PblBoardConfig *board_config);
void pebble_connect_uarts_stm32f7xx(Stm32F7xxUart *uart[], const PblBoardConfig *board_config);
void pebble_init_buttons(Stm32Gpio *gpio[], const PblButtonMap *map);
DeviceState *pebble_init_board(Stm32Gpio *gpio[], qemu_irq display_vibe);

