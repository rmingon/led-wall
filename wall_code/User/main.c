#include "debug.h"
#include <string.h>
#include <stdint.h>

#define NUM_LEDS  2   // change as needed

#define BIT_TICKS   60u
#define TICKS_0     18u     // ~0.375 us based on 48mhz
#define TICKS_1     38u     // ~0.792 us

static uint8_t  grb[NUM_LEDS][3];
static uint16_t ccr_seq[NUM_LEDS * 24];

static void encode_grb_to_ccr(void){
    size_t k = 0;
    for (size_t i = 0; i < NUM_LEDS; ++i){
        uint8_t G = grb[i][0], R = grb[i][1], B = grb[i][2];
        const uint8_t c[3] = { G, R, B };      // WS2812 expects GRB order
        for (int ch = 0; ch < 3; ++ch){
            uint8_t v = c[ch];
            for (int b = 7; b >= 0; --b){
                ccr_seq[k++] = ((v >> b) & 1) ? TICKS_1 : TICKS_0;
            }
        }
    }
}

static void ws_ll_init(void){
    RCC->PB2PCENR |= (1u<<0) | (1u<<4);   // AFIOEN, IOPCEN
    RCC->PB1PCENR |= (1u<<0);             // TIM2EN
    RCC->HBPCENR  |= (1u<<0);             // DMA1EN

    /* TIM2 remap: CH2 -> PC7  (TIM2_RM[2:0] at AFIO->PCFR1[16:14] = 0b011) */
    AFIO->PCFR1 = (AFIO->PCFR1 & ~(0x7u << 14)) | (0x3u << 14);

    /* PC7 as AF push-pull, high speed */
    GPIOC->CFGLR &= ~(0xFu << (7*4));
    GPIOC->CFGLR |=  (0xBu << (7*4));     

    TIM2->PSC   = 0;
    TIM2->ATRLR = BIT_TICKS - 1;
    TIM2->CNT   = 0;

    TIM2->CHCTLR1 &= ~(0xFFu << 8);
    TIM2->CHCTLR1 |=  (6u << 12);   // OC2M = 110 (PWM1)
    TIM2->CHCTLR1 |=  (1u << 11);   // OC2PE = 1 (preload)
    TIM2->CCER    &= ~(1u << 5);    // CC2P = 0
    TIM2->CCER    |=  (1u << 4);    // CC2E = 1

    TIM2->CTLR1   |=  (1u << 7);    // ARPE
    TIM2->SWEVGR   =  1u;           // UG (latch ARR/CCR)

    TIM2->DMAINTENR |= (1u << 8);   // UDE
}

void WS_SetPixel(size_t i, uint8_t r, uint8_t g, uint8_t b){
    if (i >= NUM_LEDS) return;
    grb[i][0] = g;
    grb[i][1] = r;
    grb[i][2] = b;
}

void WS_Show(void){
    encode_grb_to_ccr();
    
    DMA1_Channel2->CFGR &= ~1u;

    DMA1_Channel2->PADDR = (uint32_t)&TIM2->CH2CVR;  // peripheral = CCR2
    DMA1_Channel2->MADDR = (uint32_t)ccr_seq;        // memory = duty sequence
    DMA1_Channel2->CNTR  = (uint16_t)(NUM_LEDS * 24);

    /* mem->periph, 16-bit, MINC, normal, high prio */
    uint32_t cfg = 0;
    cfg |= (0u << 14);  // MEM2MEM=0
    cfg |= (2u << 12);  // PL=High
    cfg |= (1u << 10);  // MSIZE=16-bit
    cfg |= (1u <<  8);  // PSIZE=16-bit
    cfg |= (1u <<  7);  // MINC=1
    cfg |= (0u <<  6);  // PINC=0
    cfg |= (1u <<  4);  // DIR=1 (mem->periph)
    cfg |= (0u <<  3);  // CIRC=0 (one-shot)
    cfg |= (0u <<  1);  // TCIE=0 (poll)
    DMA1_Channel2->CFGR = cfg;

    DMA_ClearFlag(DMA1_FLAG_TC2);
    DMA1_Channel2->CFGR |= 1u;              // EN=1

    TIM2->SWEVGR = 1u;                      // UG
    TIM2->CTLR1 |= 1u;                      // CEN=1

    while (DMA_GetFlagStatus(DMA1_FLAG_TC2) == RESET) {}
    DMA_ClearFlag(DMA1_FLAG_TC2);

    DMA1_Channel2->CFGR &= ~1u;             // EN=0 (disable ch2)
    TIM2->DMAINTENR     &= ~(1u << 8);      // UDE=0 (no update DMA requests)

    TIM2->CH2CVR = 0;                        // duty=0 -> output low
    TIM2->SWEVGR = 1u;                       // latch CCR=0 at next cycle

    Delay_Us(100);

    TIM2->CTLR1 &= ~1u;                      // CEN=0
    TIM2->DMAINTENR |= (1u << 8);            // re-arm UDE for next frame
}

int main(void){
    SystemInit();
    Delay_Init();

    ws_ll_init();

    while (1){
        WS_SetPixel(0, 255, 0, 0);  WS_Show();  Delay_Ms(500);  // Red
        WS_SetPixel(0, 0, 0, 255);  WS_Show();  Delay_Ms(500);  // Blue
        WS_SetPixel(0, 0, 0, 0);    WS_Show();  Delay_Ms(500);  // Off
        WS_SetPixel(1, 255, 0, 0);  WS_Show();  Delay_Ms(500);  // Red
        WS_SetPixel(1, 0, 0, 255);  WS_Show();  Delay_Ms(500);  // Blue
        WS_SetPixel(1, 0, 0, 0);    WS_Show();  Delay_Ms(500);  // Off
    }
}