
#include <stdbool.h>

#include "hal.h"
#include "asf.h"

volatile bool flag;
int ticks;
int toggle;

tPinDef led_pin = PIN_DEF (1, 27, 0);

void app_SysTick (void)
{
  ticks++;
  if (ticks == 500)
  {
    flag = true;
    ticks = 0;

    if (toggle)
      write_pin (led_pin, 1);
    else
      write_pin (led_pin, 0);
    toggle = !toggle;

  }
}

void write_str (int uart_num, char *str)
{
  while (*str)
    hal_uart_send (uart_num, *str++);
}

void main()
{

  hal_init();

  set_pin_mode (led_pin, OUTPUT);

  hal_uart_init (0);

  flag = 1;

  while (1)
  {
    if (flag)
    {
      write_str (0,"hello\n");
      flag = false;
    }
  }
}