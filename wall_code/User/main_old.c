#include "ch32v00x.h"

/* ---------- user config ---------- */
#define WS_PIN_PORT   GPIOC
#define WS_PIN        GPIO_Pin_7     // PC7
#define LED_COUNT     1              // change to your strip length

#define BLINK_LED_PORT GPIOD
#define BLINK_LED_PIN  GPIO_Pin_3    // PD3 sink mode (active low)
/* --------------------------------- */

/* At 48 MHz: one timer tick ~ 20.833 ns; WS2812 bit period = 1.25 µs = 60 ticks */
#define WS_TICKS_BIT  60
#define WS_TICKS_T0H  17   // ~0.35 µs high
#define WS_TICKS_T1H  34   // ~0.70 µs high
#define WS_RESET_US   300  // reset latch time (µs)

/* Simple color helper: GRB order for WS2812 */
typedef struct { uint8_t g, r, b; } grb_t;
static grb_t leds[LED_COUNT];

/* ---- GPIO and TIM2 (free-running 48 MHz counter) ---- */
static void WS_IO_Init(void)
{
    // CH32V006 uses different RCC enable for GPIO
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOC, ENABLE);  // Enable GPIOC specifically
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOD, ENABLE);  // Enable GPIOC specifically

    GPIO_InitTypeDef io = {0};
    io.GPIO_Pin   = WS_PIN;
    io.GPIO_Speed = GPIO_Speed_30MHz;   // CH32V006 supports up to 50MHz
    io.GPIO_Mode  = GPIO_Mode_Out_PP;   // push-pull
    GPIO_Init(WS_PIN_PORT, &io);

    // idle low per WS2812 datasheet
    GPIO_ResetBits(WS_PIN_PORT, WS_PIN);

    io.GPIO_Pin   = BLINK_LED_PIN;
    io.GPIO_Speed = GPIO_Speed_30MHz;    // Lower speed for LED
    io.GPIO_Mode  = GPIO_Mode_Out_PP;   // push-pull
    GPIO_Init(BLINK_LED_PORT, &io);
    // Start with LED off (high = off in sink mode)
    GPIO_SetBits(BLINK_LED_PORT, BLINK_LED_PIN);
}

static void TIM2_Timebase_48M(void)
{
    // Fixed typo: RCC_APB1PeriphClockCmd (not RCC_PB1PeriphClockCmd)
    RCC_PB2PeriphClockCmd(RCC_PB1Periph_TIM2, ENABLE);

    TIM_TimeBaseInitTypeDef tb = {0};
    tb.TIM_Prescaler     = 0;          // 48 MHz
    tb.TIM_CounterMode   = TIM_CounterMode_Up;
    tb.TIM_Period        = 0xFFFF;     // free-running
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &tb);
    TIM_Cmd(TIM2, ENABLE);
}

/* ---- low-level timed bit send using TIM2->CNT for precise edges ---- */
static inline void ws_delay_ticks(uint16_t start, uint16_t ticks)
{
    // Wait until (TIM2->CNT - start) wraps-safe reaches 'ticks'
    while ((uint16_t)(TIM2->CNT - start) < ticks) { 
        __asm__ volatile("nop"); // Better than empty asm for CH32V RISC-V
    }
}

#define WS_HIGH()  (WS_PIN_PORT->BSHR = WS_PIN)   // set bit
#define WS_LOW()   (WS_PIN_PORT->BCR  = WS_PIN)   // clear bit

static inline void ws_send_bit(uint8_t bit)
{
    WS_HIGH();
    uint16_t t_start = TIM2->CNT;                                // start AFTER rising edge
    ws_delay_ticks(t_start, bit ? WS_TICKS_T1H : WS_TICKS_T0H);
    WS_LOW();
    ws_delay_ticks(t_start, WS_TICKS_BIT);
}

/* Safe delay function using TIM2 */
static void delay_us_safe(uint32_t microseconds)
{
    if (microseconds == 0) return;
    
    uint32_t total_ticks = microseconds * 48; // 48 MHz = 48 ticks per µs
    
    while (total_ticks > 0) {
        uint16_t start = TIM2->CNT;
        uint32_t chunk = (total_ticks > 30000) ? 30000 : total_ticks; // Max ~625µs chunks
        uint16_t target = start + (uint16_t)chunk;
        
        // Wait for timer to reach target, handling 16-bit wraparound
        uint16_t timeout = 0;
        while (1) {
            uint16_t current = TIM2->CNT;
            uint16_t elapsed = current - start;
            
            if (elapsed >= chunk) break;
            
            // Safety timeout to prevent infinite loop
            if (++timeout > 65000) break;
        }
        
        total_ticks -= chunk;
    }
}


static void ws_send_byte(uint8_t v)
{
    for (int i = 7; i >= 0; --i) {
        ws_send_bit((v >> i) & 1);
    }
}

static void ws_show(const grb_t *buf, int n)
{
    // Interrupts off during waveform to avoid timing jitter
    __disable_irq();
    for (int i = 0; i < n; ++i) {
        ws_send_byte(buf[i].g);
        ws_send_byte(buf[i].r);
        ws_send_byte(buf[i].b);
    }
    __enable_irq();

    delay_us_safe(WS_RESET_US);
}

static void ws_set_all(uint8_t r, uint8_t g, uint8_t b)
{
    for (int i = 0; i < LED_COUNT; ++i) {
        leds[i].r = r;
        leds[i].g = g;
        leds[i].b = b;
    }
}

static void blink_led_toggle(void)
{
    if (GPIO_ReadOutputDataBit(BLINK_LED_PORT, BLINK_LED_PIN)) {
        GPIO_ResetBits(BLINK_LED_PORT, BLINK_LED_PIN);  // Turn LED on
    } else {
        GPIO_SetBits(BLINK_LED_PORT, BLINK_LED_PIN);    // Turn LED off
    }
}

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();
    WS_IO_Init();
    TIM2_Timebase_48M();

    GPIO_ResetBits(BLINK_LED_PORT, BLINK_LED_PIN);
    delay_us_safe(2000000);
    while (1) {
        blink_led_toggle();

        ws_set_all(0, 10, 0);
        ws_show(leds, LED_COUNT);


        delay_us_safe(500000);
        ws_set_all(0, 0, 10);
        ws_show(leds, LED_COUNT);


        delay_us_safe(500000);
    }
}