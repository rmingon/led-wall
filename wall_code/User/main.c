#include "debug.h"
#include <string.h>
#include <stdint.h>

/* ===================== User config ===================== */
#define NUM_LEDS   25     // set to your strip length

/* === Parallel bus pins ===
 * Data:  D0..D6 -> PC0..PC6,  D7 -> PD0
 * Clock: PD2 (rising edge samples D[7:0])
 */

/* ===================== WS2812 (TIM2+DMA on PC7) ===================== */
/* 48 MHz sysclk; TIM2_CH2 -> PC7 via remap (TIM2_RM=0b011), Update-DMA -> CCR2 */
#define BIT_TICKS   60u        // 1.25 us @ 48 MHz
#define TICKS_0     18u        // ~0.375 us high
#define TICKS_1     38u        // ~0.792 us high

static uint8_t  ws_grb[NUM_LEDS][3];
static uint16_t ws_ccr[NUM_LEDS * 24];

static void ws_encode(void){
    size_t k=0;
    for (size_t i=0;i<NUM_LEDS;i++){
        uint8_t G=ws_grb[i][0], R=ws_grb[i][1], B=ws_grb[i][2];
        const uint8_t c[3]={G,R,B};
        for (int ch=0; ch<3; ch++){
            uint8_t v=c[ch];
            for (int b=7;b>=0;b--){
                ws_ccr[k++] = ((v>>b)&1) ? TICKS_1 : TICKS_0;
            }
        }
    }
}

static void WS_LL_Init(void){
    /* Clocks: AFIO, GPIOC, GPIOD, TIM2, DMA1 */
    RCC->PB2PCENR |= (1u<<0) | (1u<<4) | (1u<<5);  // AFIOEN | IOPCEN | IOPDEN
    RCC->PB1PCENR |= (1u<<0);                      // TIM2EN
    RCC->HBPCENR  |= (1u<<0);                      // DMA1EN

    /* Remap TIM2_CH2 -> PC7: AFIO->PCFR1[16:14] = 0b011 */
    AFIO->PCFR1 = (AFIO->PCFR1 & ~(0x7u << 14)) | (0x3u << 14);

    /* PC7 as AF push-pull (TIM2_CH2) */
    GPIOC->CFGLR &= ~(0xFu << (7*4));
    GPIOC->CFGLR |=  (0xBu << (7*4));   // MODE=11 (50 MHz), CNF=10 (AF-PP)

    /* TIM2 base: PSC=0 -> 48 MHz; ARR=59 -> 1.25 us period */
    TIM2->PSC   = 0;
    TIM2->ATRLR = BIT_TICKS - 1;
    TIM2->CNT   = 0;

    /* CH2 PWM1 + preload */
    TIM2->CHCTLR1 &= ~(0xFFu << 8);
    TIM2->CHCTLR1 |=  (6u << 12);   // PWM1
    TIM2->CHCTLR1 |=  (1u << 11);   // OC2PE
    TIM2->CCER    &= ~(1u << 5);    // active-high
    TIM2->CCER    |=  (1u << 4);    // CC2E enable

    TIM2->CTLR1   |=  (1u << 7);    // ARPE
    TIM2->SWEVGR   =  1u;           // UG

    /* Update-DMA request */
    TIM2->DMAINTENR |= (1u << 8);   // UDE
}

static void WS_DMA_Start(uint16_t *src, uint16_t count){
    DMA1_Channel2->CFGR &= ~1u;  // EN=0
    DMA1_Channel2->PADDR = (uint32_t)&TIM2->CH2CVR;  // dest = CCR2
    DMA1_Channel2->MADDR = (uint32_t)src;
    DMA1_Channel2->CNTR  = count;

    uint32_t cfg=0;
    cfg |= (0u<<14);  // MEM2MEM=0
    cfg |= (2u<<12);  // High priority
    cfg |= (1u<<10);  // MSIZE=16-bit
    cfg |= (1u<< 8);  // PSIZE=16-bit
    cfg |= (1u<< 7);  // MINC=1
    cfg |= (0u<< 6);  // PINC=0
    cfg |= (1u<< 4);  // DIR=1 (mem->periph)
    cfg |= (0u<< 3);  // CIRC=0
    cfg |= (0u<< 1);  // TCIE=0
    DMA1_Channel2->CFGR = cfg;

    DMA_ClearFlag(DMA1_FLAG_TC2);
    DMA1_Channel2->CFGR |= 1u; // EN=1
}

static void WS_Show(void){
    ws_encode();

    WS_DMA_Start(ws_ccr, (uint16_t)(NUM_LEDS*24));

    TIM2->SWEVGR = 1u;   // UG
    TIM2->CTLR1 |= 1u;   // CEN

    while (DMA_GetFlagStatus(DMA1_FLAG_TC2) == RESET) {}
    DMA_ClearFlag(DMA1_FLAG_TC2);

    DMA1_Channel2->CFGR &= ~1u;        // EN=0
    TIM2->DMAINTENR     &= ~(1u<<8);   // UDE=0
    TIM2->CH2CVR = 0;                  // duty=0
    TIM2->SWEVGR = 1u;                 // latch low
    Delay_Us(100);                     // >80 us reset
    TIM2->CTLR1 &= ~1u;                // stop timer
    TIM2->DMAINTENR |= (1u<<8);        // re-arm UDE
}

static inline void WS_SetPixel_RGB(size_t i, uint8_t r, uint8_t g, uint8_t b){
    if (i >= NUM_LEDS) return;
    ws_grb[i][0] = g;
    ws_grb[i][1] = r;
    ws_grb[i][2] = b;
}

static volatile uint8_t  rx_buf[NUM_LEDS * 3];
static volatile uint16_t rx_idx = 0;
static volatile uint8_t  frame_ready = 0;

static inline uint8_t read_parallel_byte(void){
    uint32_t c = GPIOC->INDR;    // input data
    uint32_t d = GPIOD->INDR;
    uint8_t v  = (uint8_t)(c & 0x7Fu);       // bits 0..6
    v |= (uint8_t)((d & 0x1u) << 7);         // PD0 -> bit 7
    return v;
}

void EXTI7_0_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line2) != RESET) {
        if (!frame_ready){                        // ignore extra bytes while main is busy
            uint8_t b = read_parallel_byte();
            rx_buf[rx_idx++] = b;
            if (rx_idx >= (NUM_LEDS * 3)){
                frame_ready = 1;                  // full RGB frame received
                EXTI->INTENR &= ~(1u<<2);        // optionally gate clock until consumed
            }
        }
        EXTI_ClearITPendingBit(EXTI_Line2);
    }
}

static void ParallelBus_Init(void){
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_AFIO |
                           RCC_PB2Periph_GPIOC |
                           RCC_PB2Periph_GPIOD, ENABLE);

    GPIO_InitTypeDef io = {0};

    /* PC0..PC6: input pull-up */
    io.GPIO_Pin  = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |
                   GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
    io.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOC, &io);

    /* PD0 (D7) + PD2 (CLK): input pull-up */
    io.GPIO_Pin  = GPIO_Pin_0 | GPIO_Pin_2;
    io.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOD, &io);

    /* EXTI line 2 on PD2, rising edge */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource2);
    EXTI_InitTypeDef ex = {0};
    ex.EXTI_Line    = EXTI_Line2;
    ex.EXTI_Mode    = EXTI_Mode_Interrupt;
    ex.EXTI_Trigger = EXTI_Trigger_Rising;
    ex.EXTI_LineCmd = ENABLE;
    EXTI_Init(&ex);

    NVIC_InitTypeDef nv = {0};
    nv.NVIC_IRQChannel                   = EXTI7_0_IRQn;
    nv.NVIC_IRQChannelPreemptionPriority = 1;
    nv.NVIC_IRQChannelSubPriority        = 1;
    nv.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nv);
}

static void WS_BootTest(void) {
    for (size_t i = 0; i < NUM_LEDS; i++) {
        // Turn all off first
        for (size_t j = 0; j < NUM_LEDS; j++) {
            WS_SetPixel_RGB(j, 0, 0, 0);
        }

        // Red
        WS_SetPixel_RGB(i, 255, 0, 0);
        WS_Show();
        Delay_Ms(300);

        // Green
        WS_SetPixel_RGB(i, 0, 255, 0);
        WS_Show();
        Delay_Ms(300);

        // Blue
        WS_SetPixel_RGB(i, 0, 0, 255);
        WS_Show();
        Delay_Ms(300);

        // Off
        WS_SetPixel_RGB(i, 0, 0, 0);
        WS_Show();
        Delay_Ms(200);
    }
}

static void ParallelBus_ProcessFrame(void){
    for (size_t i=0, j=0; i<NUM_LEDS; i++, j+=3){
        uint8_t r = rx_buf[j+0];
        uint8_t g = rx_buf[j+1];
        uint8_t b = rx_buf[j+2];
        WS_SetPixel_RGB(i, r, g, b);
    }
    WS_Show();

    rx_idx = 0;
    frame_ready = 0;
    EXTI->INTENR |= (1u<<2);     // re-enable EXTI2 if you gated it
}

/* ===================== Demo main ===================== */
int main(void){
    SystemInit();
    Delay_Init();

    WS_LL_Init();         // PC7 WS2812 driver
    ParallelBus_Init();   // PC0..6, PD0 data; PD2 clock

     WS_BootTest();        // <<< run self-tes

    /* Optional: light something until host sends a frame */
    WS_SetPixel_RGB(0, 0, 0, 0);
    WS_Show();

    while (1){
        if (frame_ready){
            ParallelBus_ProcessFrame();
        }
        /* low-power hints:
           __WFI();  // wake on EXTI2 IRQ
        */
    }
}