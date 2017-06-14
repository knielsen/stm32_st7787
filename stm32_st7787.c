/*
  Talk to ST7787 display controller.

  Pinout:
    CS       PB12            VDDI --
    DC       PB13           RESET -- PC15
    WR       PB14             IM0 -- PC14
    RD       PB15             IM1 -- PC13
    DB0-15   PD0-15           IM2 -- PC12
    DB16     PC9               TE -- PC11
    DB17     PC10            DB17 -- PC10
    IM0      PC14            DB16 -- PC9
    IM1      PC13            DB15 -- PD15
    IM2      PC12            DB14 -- PD14
    TE       PC11            DB13 -- PD13
    RESET    PC15            DB12 -- PD12
                             DB11 -- PD11
                             DB10 -- PD10
                              DB9 -- PD9
                              DB8 -- PD8
                              DB7 -- PD7
                              DB6 -- PD6
                              DB5 -- PD5
                              DB4 -- PD4
                              DB3 -- PD3
                              DB2 -- PD2
                              DB1 -- PD1
                              DB0 -- PD0
                               RD -- PB15
                               WR -- PB14
                               DC -- PB13
                               CS -- PB12
                              GND --
                              VDD --
                        (LED-)GND --
                             LED+ --
*/

#include <math.h>

#include <stm32f4xx.h>

#define MCU_HZ 168000000

static void delay(uint32_t nCount)
{
  /* This should be 3 cycles per iteration. nCount must be > 0. */
  __asm volatile
    ("\n"
     "0:\n\t"
     "subs %1, #1\n\t"
     "bne.n 0b"
     : "=r" (nCount)
     : "0" (nCount)
     : "cc");
}


static void setup_serial(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;

  /* enable peripheral clock for USART6 */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

  /* GPIOG clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);

  /* GPIOG Configuration:  USART6 TX on PG14 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOG, &GPIO_InitStructure);

  /* Connect USART6 pins to AF8 */
  // TX = PG14
  GPIO_PinAFConfig(GPIOG, GPIO_PinSource14, GPIO_AF_USART6);

  USART_InitStructure.USART_BaudRate = 500000;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx;
  USART_Init(USART6, &USART_InitStructure);

  USART_Cmd(USART6, ENABLE); // enable USART6
}


static void
serial_putchar(USART_TypeDef* USARTx, uint32_t c)
{
  while(!(USARTx->SR & USART_FLAG_TC));
  USART_SendData(USARTx, c);
}


static void
serial_puts(USART_TypeDef *usart, const char *s)
{
  while (*s)
    serial_putchar(usart, (uint8_t)*s++);
}


static void
serial_output_hexdig(USART_TypeDef* USARTx, uint32_t dig)
{
  serial_putchar(USARTx, (dig >= 10 ? 'A' - 10 + dig : '0' + dig));
}


__attribute__ ((unused))
static void
serial_output_hexbyte(USART_TypeDef* USARTx, uint8_t byte)
{
  serial_output_hexdig(USARTx, byte >> 4);
  serial_output_hexdig(USARTx, byte & 0xf);
}


__attribute__ ((unused))
static void
serial_output_hex(USART_TypeDef* USARTx, uint32_t v)
{
  serial_putchar(USARTx, '0');
  serial_putchar(USARTx, 'x');
  serial_output_hexbyte(USARTx, v >> 24);
  serial_output_hexbyte(USARTx, (v >> 16) & 0xff);
  serial_output_hexbyte(USARTx, (v >> 8) & 0xff);
  serial_output_hexbyte(USARTx, v & 0xff);
}


__attribute__ ((unused))
static char *
tostring_uint32(char *p, uint32_t val)
{
  uint32_t l, d;

  l = 1000000000UL;
  while (l > val && l > 1)
    l /= 10;

  do
  {
    d = val / l;
    *p++ = '0' + d;
    val -= d*l;
    l /= 10;
  } while (l > 0);
  return p;
}


__attribute__ ((unused))
static void
print_uint32(USART_TypeDef* usart, uint32_t val)
{
  char buf[13];
  char *p;

  p = tostring_uint32(buf, val);
  *p = '\0';
  serial_puts(usart, buf);
}


__attribute__ ((unused))
static void
println_uint32(USART_TypeDef* usart, uint32_t val)
{
  char buf[13];
  char *p = buf;

  p = tostring_uint32(buf, val);
  *p++ = '\r';
  *p++ = '\n';
  *p = '\0';
  serial_puts(usart, buf);
}


__attribute__ ((unused))
static void
println_int32(USART_TypeDef* usart, int32_t val)
{
  if (val < 0)
  {
    serial_putchar(usart, '-');
    println_uint32(usart, (uint32_t)0 - (uint32_t)val);
  }
  else
    println_uint32(usart, val);
}


static void
float_to_str(char *buf, float f, uint32_t dig_before, uint32_t dig_after)
{
  float a;
  uint32_t d;
  uint8_t leading_zero;

  if (f == 0.0f)
  {
    buf[0] = '0';
    buf[1] = '\0';
    return;
  }
  if (f < 0)
  {
    *buf++ = '-';
    f = -f;
  }
  a =  powf(10.0f, (float)dig_before);
  if (f >= a)
  {
    buf[0] = '#';
    buf[1] = '\0';
    return;
  }
  leading_zero = 1;
  while (dig_before)
  {
    a /= 10.0f;
    d = (uint32_t)(f / a);
    if (leading_zero && d == 0 && a >= 10.0f)
      *buf++ = ' ';
    else
    {
      leading_zero = 0;
      *buf++ = '0' + d;
      f -= d*a;
    }
    --dig_before;
  }
  if (!dig_after)
  {
    *buf++ = '\0';
    return;
  }
  *buf++ = '.';
  do
  {
    f *= 10.0f;
    d = (uint32_t)f;
    *buf++ = '0' + d;
    f -= (float)d;
    --dig_after;
  } while (dig_after);
  *buf++ = '\0';
}


__attribute__ ((unused))
static void
println_float(USART_TypeDef* usart, float f,
              uint32_t dig_before, uint32_t dig_after)
{
  char buf[21];
  char *p = buf;

  float_to_str(p, f, dig_before, dig_after);
  while (*p)
    ++p;
  *p++ = '\r';
  *p++ = '\n';
  *p = '\0';
  serial_puts(usart, buf);
}


#define LED_GPIO_PERIPH RCC_AHB1Periph_GPIOG
#define LED_GPIO GPIOG
#define LED_PIN GPIO_Pin_15


static void
setup_leds(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(LED_GPIO_PERIPH, ENABLE);
  GPIO_InitStructure.GPIO_Pin = LED_PIN;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(LED_GPIO, &GPIO_InitStructure);
}


__attribute__((unused))
static void
led_on(void)
{
  GPIO_SetBits(LED_GPIO, LED_PIN);
}


__attribute__((unused))
static void
led_off(void)
{
  GPIO_ResetBits(LED_GPIO, LED_PIN);
}


static void
setup_display_io(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  /* CS/DC/WR/RD on PB12-15 as output. */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15|GPIO_Pin_14|GPIO_Pin_13|GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  /* Initialise CS/WR/RD all de-asserted, DC as command. */
  GPIO_SetBits(GPIOB, GPIO_Pin_12|GPIO_Pin_14|GPIO_Pin_15);
  GPIO_ResetBits(GPIOB, GPIO_Pin_13);

  /* IM2/IM1/IM0/RESET on PC12-15 as output. */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15|GPIO_Pin_14|GPIO_Pin_13|GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  /* Assert RESET, and select 16-bit parallel interface on IM0-2. */
  GPIO_ResetBits(GPIOC, GPIO_Pin_15);
  GPIO_ResetBits(GPIOC, GPIO_Pin_13);     /* IM1 <- 0   8/16 over 9/18 bits */
  GPIO_SetBits(GPIOC, GPIO_Pin_14);       /* IM0 <- 1   16 over 8 bits */
  GPIO_SetBits(GPIOC, GPIO_Pin_12);       /* IM2 <- 1   parallel interface */

  /* TE on PC11 as input. PB16-17 on PC9-10 as input (for now). */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* PB0-15 on PD0-15 as input (for now). */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|
    GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|
    GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  /* After power-up, should hold RESET asserted for min 120 msec. */
  delay(MCU_HZ/1000*120/3);
  /* Now release reset, taking the device into operational mode. */
  GPIO_SetBits(GPIOC, GPIO_Pin_15);
}


static inline void
assert_rd(void)
{
  GPIO_ResetBits(GPIOB, GPIO_Pin_15);
}


static inline void
deassert_rd(void)
{
  GPIO_SetBits(GPIOB, GPIO_Pin_15);
}


static inline void
assert_wr(void)
{
  GPIO_ResetBits(GPIOB, GPIO_Pin_14);
}


static inline void
deassert_wr(void)
{
  GPIO_SetBits(GPIOB, GPIO_Pin_14);
}


static inline void
assert_cs(void)
{
  GPIO_ResetBits(GPIOB, GPIO_Pin_12);
}


static inline void
deassert_cs(void)
{
  GPIO_SetBits(GPIOB, GPIO_Pin_12);
}


static inline void
dc_select_command(void)
{
  GPIO_ResetBits(GPIOB, GPIO_Pin_13);
}


static inline void
dc_select_data(void)
{
  GPIO_SetBits(GPIOB, GPIO_Pin_13);
}


static void
db_select_input(void)
{
  GPIO_InitTypeDef gpio;
  gpio.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3| GPIO_Pin_4|
    GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|
    GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
  gpio.GPIO_Mode  = GPIO_Mode_IN;
  gpio.GPIO_Speed = GPIO_Speed_50MHz;
  gpio.GPIO_OType = GPIO_OType_PP;
  gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &gpio);
  /*
    ToDo: Do we need to handle DB16-DB17 also? Or can they be left floating,
    or grounded perhaps?
  */
}


static void
db_select_output(void)
{
  GPIO_InitTypeDef gpio;
  gpio.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3| GPIO_Pin_4|
    GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|
    GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
  gpio.GPIO_Mode  = GPIO_Mode_OUT;
  gpio.GPIO_Speed = GPIO_Speed_50MHz;
  gpio.GPIO_OType = GPIO_OType_PP;
  gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &gpio);
  /*
    ToDo: Do we need to handle DB16-DB17 also? Or can they be left floating,
    or grounded perhaps?
  */
}


static inline void
db_write16(uint32_t value)
{
  GPIOD->ODR = value & 0xffff;
}


static inline uint32_t
db_read16(void)
{
  return GPIOD->IDR & 0xffff;
}


int
main()
{
  setup_serial();
  setup_leds();
  setup_display_io();
  serial_puts(USART6, "Initialisation done.\r\n");
  delay(2000000);

  for (;;) {
    led_on();
    db_select_output();
    delay(MCU_HZ/3/2);
    led_off();
    db_select_input();
    delay(MCU_HZ/3/2);
  }
}
