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
#include <string.h>

#include <stm32f4xx.h>

#include "st7787.h"


#define MODE_16BIT
//#define MODE_1BIT

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


static inline void
delay_ns(uint32_t ns)
{
  delay((MCU_HZ/(3*1000000)*ns+999)/1000);
}


static inline void
delay_ms(uint32_t ms)
{
  delay(MCU_HZ/(3*1000)*ms);
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
  /* Assert RESET. */
  GPIO_ResetBits(GPIOC, GPIO_Pin_15);
#ifdef MODE_16BIT
  /* Select 16-bit parallel interface on IM0-2. */
  GPIO_ResetBits(GPIOC, GPIO_Pin_13);     /* IM1 <- 0   8/16 over 9/18 bits */
  GPIO_SetBits(GPIOC, GPIO_Pin_14);       /* IM0 <- 1   16 over 8 bits */
  GPIO_SetBits(GPIOC, GPIO_Pin_12);       /* IM2 <- 1   parallel interface */
#endif
#ifdef MODE_6BIT
  /* Select 1-bit serial interface on IM0-2. */
  GPIO_ResetBits(GPIOC, GPIO_Pin_13);     /* IM1 <- 0   (8/16 over 9/18 bits) */
  GPIO_ResetBits(GPIOC, GPIO_Pin_14);     /* IM0 <- 0   (8 over 16 bits) */
  GPIO_ResetBits(GPIOC, GPIO_Pin_12);     /* IM2 <- 0   serial interface */
#endif

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

  /*
    After power-up, should hold RESET asserted for min 120 msec.
    Let do a 120 ms reset pulse, followed by a 20 us reset pulse.
    (The datasheet was a bit unclear, not sure if this wait is needed, or
    if a 20 us reset pulse is enough).
  */
  GPIO_SetBits(GPIOC, GPIO_Pin_15);
  delay_ms(1);
  GPIO_ResetBits(GPIOC, GPIO_Pin_15);
  delay_ms(120);
  GPIO_SetBits(GPIOC, GPIO_Pin_15);
  delay_ms(1);
  GPIO_ResetBits(GPIOC, GPIO_Pin_15);
  delay_ns(20*1000);
  /* Now release reset, taking the device into operational mode. */
  GPIO_SetBits(GPIOC, GPIO_Pin_15);
  /* Wait anoter 120 msec for RESET to complete. */
  delay_ms(120);
}


#ifdef MODE_16BIT
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
#endif


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
#ifdef MODE_16BIT
dc_select_command(void)
#endif
#ifdef MODE_1BIT
scl_low(void)
#endif
{
  GPIO_ResetBits(GPIOB, GPIO_Pin_13);
}


static inline void
#ifdef MODE_16BIT
dc_select_data(void)
#endif
#ifdef MODE_1BIT
scl_high(void)
#endif
{
  GPIO_SetBits(GPIOB, GPIO_Pin_13);
}


#ifdef MODE_16BIT
static void
db_select_input_16(void)
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
db_select_output_16(void)
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
#endif


#ifdef MODE_1BIT
static void
db_select_input_1(void)
{
  GPIO_InitTypeDef gpio;
  gpio.GPIO_Pin = GPIO_Pin_0;
  gpio.GPIO_Mode  = GPIO_Mode_IN;
  gpio.GPIO_Speed = GPIO_Speed_50MHz;
  gpio.GPIO_OType = GPIO_OType_PP;
  gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &gpio);
  /*
    ToDo: DB1-17 can be tied to GND or VCC, according to datasheet.
    But for now, let's leave them floating, to avoid risk of contention between
    MCU and display pins both in output mode.
  */
}


static void
db_select_output_1(void)
{
  GPIO_InitTypeDef gpio;
  gpio.GPIO_Pin = GPIO_Pin_0;
  gpio.GPIO_Mode  = GPIO_Mode_OUT;
  gpio.GPIO_Speed = GPIO_Speed_50MHz;
  gpio.GPIO_OType = GPIO_OType_PP;
  gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &gpio);
}


static inline void
db_write1(uint32_t value)
{
  GPIOD->ODR = (GPIOD->ODR & ~0x1) | (value & 0x1);
}


static inline uint32_t
db_read1(void)
{
  return GPIOD->IDR & 0x1;
}
#endif


#ifdef MODE_16BIT
static void
display_command(uint8_t cmd, uint16_t *in, uint32_t in_len, uint16_t *out, uint32_t out_len)
{
  assert_cs();

  /* Write the command byte. */
  dc_select_command();
  delay_ns(T_AST);
  db_select_output_16();
  db_write16(cmd);
  assert_wr();
  delay_ns(T_DST);
  deassert_wr();
  delay_ns(T_DHT);

  /* Write any command data. */
  if (in_len) {
    dc_select_data();
    delay_ns(T_AST);
    do {
      db_write16(*in++);
      assert_wr();
      delay_ns(T_DST);
      deassert_wr();
      delay_ns(T_DHT);
    } while (--in_len > 0);
  }

  db_select_input_16();

  /* Read reply. */
  if (out_len) {
    dc_select_data();
    delay_ns(T_AST);
    do
    {
      assert_rd();
      delay_ns(T_RAT);
      *out++ = db_read16();
      delay_ns(T_RC);   /* - T_RAT */
      deassert_rd();
      delay_ns(T_RDH);
    } while (--out_len > 0);
  }

  deassert_cs();
}
#endif

#ifdef MODE_1BIT
static void
display_command(uint8_t cmd, uint16_t *in, uint32_t in_len, uint16_t *out, uint32_t out_len)
{
  uint32_t i;
  uint32_t val;

  db_select_output_1();
  assert_cs();
  delay_ns(T_CSS);

  /* Write the command byte. */
  /* First shift out the data/command bit ('0' for command). */
  scl_low();
  db_write1(0);
  delay_ns(T_SLW);
  scl_high();
  delay_ns(T_SHW);
  /* Then shift out the bits, MSB-to-LSB. */
  val = cmd;
  for (i = 8; i; --i) {
    scl_low();
    db_write1((val>>7) & 1);
    val <<= 1;
    delay_ns(T_SLW);
    scl_high();
    delay_ns(T_SHW);
  }

  /* Write any command data. */
  if (in_len) {
    do {
      val = *in++;

      /* Special case: write framebuffer data needs all 16 bits from buffer. */
      if (cmd == C_RAMWR) {
        uint32_t val2 = val >> 8;
        /* Data/command bit ('1' for data). */
        scl_low();
        db_write1(1);
        delay_ns(T_SLW);
        scl_high();
        delay_ns(T_SHW);

        for (i = 8; i; --i) {
          scl_low();
          db_write1((val2>>7) & 1);
          val2 <<= 1;
          delay_ns(T_SLW);
          scl_high();
          delay_ns(T_SHW);
        }
      }

      /* Data/command bit ('1' for data). */
      scl_low();
      db_write1(1);
      delay_ns(T_SLW);
      scl_high();
      delay_ns(T_SHW);

      for (i = 8; i; --i) {
        scl_low();
        db_write1((val>>7) & 1);
        val <<= 1;
        delay_ns(T_SLW);
        scl_high();
        delay_ns(T_SHW);
      }
    } while (--in_len > 0);
  }

  db_select_input_1();

  /* Read reply. */
  if (out_len) {
    /* Dummy bit (according to data sheet, data/command placeholder?). */
    scl_low();
    delay_ns(T_SLR);
    scl_high();
    delay_ns(T_SHR);
    /* The dummy bit seems to correspond to dummy read byte in parallel interface. */
    *out++ = 0;

    while (--out_len > 0) {
      val = 0;
      for (i = 8; i; --i) {
        scl_low();
        delay_ns(T_SLR);
        val = (val << 1) | db_read1();
        scl_high();
        delay_ns(T_SHR);
      }
      *out++ = val;
    }
  }

  scl_low();
  delay_ns(T_SCC);
  deassert_cs();
  delay_ns(T_CHW);
}
#endif


static void
display_blit(uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint16_t *pixels)
{
  uint16_t buf[4];

  buf[0] = x >> 8;
  buf[1] = x & 0xff;
  buf[2] = (x+w-1) >> 8;
  buf[3] = (x+w-1) & 0xff;
  display_command(C_CASET, buf, 4, NULL, 0);
  buf[0] = y >> 8;
  buf[1] = y & 0xff;
  buf[2] = (y+h-1) >> 8;
  buf[3] = (y+h-1) & 0xff;
  display_command(C_RASET, buf, 4, NULL, 0);
  display_command(C_RAMWR, pixels, w*h, NULL, 0);
}


static void
display_cls(void)
{
  uint16_t buf[240];
  uint32_t i;

  memset(buf, 0, sizeof(buf));
  for (i = 0; i < 320; ++i) {
    display_blit(0, i, 120, 1, buf);
    display_blit(120, i, 120, 1, buf);
  }
}


static uint16_t
mk_rgb565(uint32_t r, uint32_t g, uint32_t b)
{
  return ((r&0x1f) << 11) | ((g&0x3f) << 5) | (b&0x1f);
}


static uint32_t test_img1_counter = 0;

__attribute__((unused))
static void
test_img1(void)
{
  uint16_t buf[256];
  uint32_t i, j;

  for (i = 0; i < 16; ++i) {
    for (j = 0; j < 16; ++j) {
      buf[i*16+j] = mk_rgb565(i*2, j*4, test_img1_counter & 0x1f);
    }
  }
  i = 16*(test_img1_counter % 15);
  j = 16*((test_img1_counter/15) % 20);
  display_blit(i, j, 16, 16, buf);
  ++test_img1_counter;
}


/* Mandelbrot set */
__attribute__((unused))
static void
test_img2(void)
{
  uint32_t i, j, k;
  float p, q, x, y;

  for (j = 0; j < 320; ++j) {
    uint16_t pixels[240];
    q = ((float)j-160.0f)*(1.5f/160.0f);
    for (i = 0; i < 240; ++i) {
      p = ((float)i-200.0f)*(1.5f/120.0f);
      x = 0;
      y = 0;
      for (k = 0; k < 63; ++k) {
        float x2 = x*x;
        float y2 = y*y;
        float xy = x*y;
        if (x2+y2 >= 4)
          break;
        x = x2 - y2 + p;
        y = 2*xy + q;
      }
      if (k < 63)
        pixels[i] = mk_rgb565(0, k+1, 0);
      else
        pixels[i] = mk_rgb565(0, 0, 0);
    }
    display_blit(0, j, 240, 1, pixels);
  }
}


/* Photo of a chipmonk. */
#define gimp_image test_img3_chipmonk_data
#include "chipmunk_herbs_nuts_83281_240x320.c"
__attribute__((unused))
static void
test_img3(void)
{
  display_blit(0, 0, gimp_image.width, gimp_image.height, (uint16_t*)gimp_image.pixel_data);
}
#undef gimp_image


int
main()
{
  uint16_t buf[5];

  setup_serial();
  setup_leds();
  setup_display_io();
  serial_puts(USART6, "Initialisation done.\r\n");
  delay(2000000);

  /* Take the display out of sleep mode. */
  display_command(C_SLPOUT, NULL, 0, NULL, 0);
  /*
    Wait for sleep-out command to complete.
    Datasheet says 5 msec is enough before next command, but 120 msec is
    needed before the display is fully out of sleep mode.
  */
  delay_ms(120);
  /*
    Select 16-bit 565 RGB pixel format (mode 5).
    Same for RGB mode (but we don't use it).
  */
  buf[0] = (13 << 4) | 5;
  display_command(C_COLMOD, buf, 1, NULL, 0);
  /* Clear the screen. */
  display_cls();
  /* Disable external vsync. */
  display_command(C_VSYNCOUT, NULL, 0, NULL, 0);
  /* Turn on the display */
  display_command(C_DISPON, NULL, 0, NULL, 0);

  for (;;) {
    uint32_t i;

    led_on();

    display_command(C_RDDID, NULL, 0, buf, 4);
    serial_puts(USART6, "RDDID:");
    for (i = 0; i < 4; ++i) {
      serial_puts(USART6, " ");
      serial_output_hexbyte(USART6, buf[i] >> 8);
      serial_output_hexbyte(USART6, buf[i] & 0xff);
    }
    serial_puts(USART6, "\r\n");

    display_command(C_RDDST, NULL, 0, buf, 5);
    serial_puts(USART6, "  RDDST:");
    for (i = 0; i < 5; ++i) {
      serial_puts(USART6, " ");
      serial_output_hexbyte(USART6, buf[i] >> 8);
      serial_output_hexbyte(USART6, buf[i] & 0xff);
    }
    serial_puts(USART6, "\r\n");

    //test_img1();
    //test_img2();
    test_img3();

    led_off();
    delay(MCU_HZ/3);
  }
}

/* Disable use of static constructors, called from ST startup code. */
void
__libc_init_array(void)
{
  /* Nothing. */
}
