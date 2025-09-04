/* ============================================================
 * CH32V006 SLAVE: WS2812 (PC7, TIM2+DMA) + 8-bit bus + address
 *  Data bus: PC0..PC6 = D0..D6, PD0 = D7
 *  Clock   : PD2 rising edge (EXTI lines 0..7 → EXTI7_0 IRQ)
 *  LEDs    : WS2812 on PC7 via TIM2_CH2 remap + DMA1 Channel2
 * ============================================================ */

#include "debug.h"
#include <stdint.h>
#include <string.h>

/* ==================== User configuration ==================== */
#define DEVICE_ADDR   0x12      /* 0..254 unique per node; 0xFF is broadcast */
#define NUM_LEDS      25        
#define BIT_TICKS   60u
#define TICKS_0     18u
#define TICKS_1     38u

/* ==================== Protocol (framed + addressed) ==========
 * Frame bytes: A5 5A ADDR CMD LEN PAY[LEN] CRC
 * ADDR: 0..254 unicast, 0xFF broadcast
 * CRC : Dallas/Maxim CRC8 (poly 0x31 reflected → 0x8C), init 0x00,
 *       over ADDR..CMD..LEN..PAYLOAD
 * Commands:
 *  0x01 SET_ALL_RGB : LEN=3*N, payload RGB triplets
 *  0x02 SET_ONE_RGB : LEN=4, payload idx,R,G,B
 *  0x03 FILL_RGB    : LEN=3, payload R,G,B
 *  0x04 BOOT_TEST   : LEN=0
 * ============================================================ */
#define SOF0  0xA5
#define SOF1  0x5A
#define ADDR_BCAST 0xFF

#define CMD_SET_ALL_RGB  0x01
#define CMD_SET_ONE_RGB  0x02
#define CMD_FILL_RGB     0x03
#define CMD_BOOT_TEST    0x04

static uint8_t  ws_grb[NUM_LEDS][3];
static uint16_t ws_ccr[NUM_LEDS * 24];

static void WS_encode(void) {
    size_t k = 0;
    for (size_t i = 0; i < NUM_LEDS; ++i) {
        /* GRB order expected by WS2812 */
        const uint8_t c[3] = { ws_grb[i][0], ws_grb[i][1], ws_grb[i][2] };
        for (int ch = 0; ch < 3; ++ch) {
            uint8_t v = c[ch];
            for (int b = 7; b >= 0; --b) {
                ws_ccr[k++] = ((v >> b) & 1) ? TICKS_1 : TICKS_0;
            }
        }
    }
}

/* Remap TIM2_CH2 → PC7; set PC7 AF-PP; configure TIM2 + UDE */
static void WS_LL_Init(void) {
    RCC->PB2PCENR |= (1u<<0) /*AFIOEN*/ | (1u<<4) /*IOPCEN*/;
    RCC->PB1PCENR |= (1u<<0) /*TIM2EN*/;
    RCC->HBPCENR  |= (1u<<0) /*DMA1EN*/;

    AFIO->PCFR1 = (AFIO->PCFR1 & ~(0x7u << 14)) | (0x3u << 14);

    /* PC7 = AF push-pull, 30 MHz */
    GPIOC->CFGLR &= ~(0xFu << (7*4));
    GPIOC->CFGLR |=  (0xBu << (7*4)); /* MODE=11, CNF=10 */

    TIM2->PSC   = 0;
    TIM2->ATRLR = BIT_TICKS - 1;
    TIM2->CNT   = 0;

    TIM2->CHCTLR1 &= ~(0xFFu << 8);
    TIM2->CHCTLR1 |=  (6u << 12); /* OC2M=110 PWM1 */
    TIM2->CHCTLR1 |=  (1u << 11); /* OC2PE=1 */
    TIM2->CCER    &= ~(1u << 5);  /* CC2P=0 */
    TIM2->CCER    |=  (1u << 4);  /* CC2E=1 */

    TIM2->CTLR1   |=  (1u << 7);  /* ARPE */
    TIM2->SWEVGR   =  1u;         /* UG */

    /* Update DMA request enable (one write to CCR2 each period) */
    TIM2->DMAINTENR |= (1u << 8); /* UDE */
}

static void WS_DMA_Start(uint16_t *src, uint16_t count) {
    DMA1_Channel2->CFGR &= ~1u; /* EN=0 */

    DMA1_Channel2->PADDR = (uint32_t)&TIM2->CH2CVR; /* dest = CCR2 */
    DMA1_Channel2->MADDR = (uint32_t)src;
    DMA1_Channel2->CNTR  = count;

    uint32_t cfg = 0;
    cfg |= (0u<<14); /* MEM2MEM=0 */
    cfg |= (2u<<12); /* High priority */
    cfg |= (1u<<10); /* MSIZE=16-bit */
    cfg |= (1u<< 8); /* PSIZE=16-bit */
    cfg |= (1u<< 7); /* MINC=1 */
    cfg |= (0u<< 6); /* PINC=0 */
    cfg |= (1u<< 4); /* DIR=1 (mem→periph) */
    cfg |= (0u<< 3); /* CIRC=0 */
    cfg |= (0u<< 1); /* TCIE=0 (poll) */
    DMA1_Channel2->CFGR = cfg;

    DMA_ClearFlag(DMA1_FLAG_TC2);
    DMA1_Channel2->CFGR |= 1u; /* EN=1 */
}

static inline void WS_SetPixel_RGB(size_t i, uint8_t r, uint8_t g, uint8_t b) {
    if (i >= NUM_LEDS) return;
    ws_grb[i][0] = g; ws_grb[i][1] = r; ws_grb[i][2] = b;
}

static void WS_BootTest(void) {
    for (size_t i = 0; i < NUM_LEDS; i++) {
        for (size_t j = 0; j < NUM_LEDS; j++) WS_SetPixel_RGB(j, 0, 0, 0);

        WS_SetPixel_RGB(i, 255, 0, 0); WS_encode(); WS_DMA_Start(ws_ccr, (uint16_t)(NUM_LEDS*24));
        TIM2->SWEVGR = 1u; TIM2->CTLR1 |= 1u;
        while (DMA_GetFlagStatus(DMA1_FLAG_TC2) == RESET) {}
        DMA_ClearFlag(DMA1_FLAG_TC2);
        DMA1_Channel2->CFGR &= ~1u; TIM2->DMAINTENR &= ~(1u<<8);
        TIM2->CH2CVR = 0; TIM2->SWEVGR = 1u; Delay_Ms(200);
        TIM2->CTLR1 &= ~1u; TIM2->DMAINTENR |= (1u<<8);

        WS_SetPixel_RGB(i, 0, 255, 0); WS_encode(); WS_DMA_Start(ws_ccr, (uint16_t)(NUM_LEDS*24));
        TIM2->SWEVGR = 1u; TIM2->CTLR1 |= 1u;
        while (DMA_GetFlagStatus(DMA1_FLAG_TC2) == RESET) {}
        DMA_ClearFlag(DMA1_FLAG_TC2);
        DMA1_Channel2->CFGR &= ~1u; TIM2->DMAINTENR &= ~(1u<<8);
        TIM2->CH2CVR = 0; TIM2->SWEVGR = 1u; Delay_Ms(200);
        TIM2->CTLR1 &= ~1u; TIM2->DMAINTENR |= (1u<<8);

        WS_SetPixel_RGB(i, 0, 0, 255); WS_encode(); WS_DMA_Start(ws_ccr, (uint16_t)(NUM_LEDS*24));
        TIM2->SWEVGR = 1u; TIM2->CTLR1 |= 1u;
        while (DMA_GetFlagStatus(DMA1_FLAG_TC2) == RESET) {}
        DMA_ClearFlag(DMA1_FLAG_TC2);
        DMA1_Channel2->CFGR &= ~1u; TIM2->DMAINTENR &= ~(1u<<8);
        TIM2->CH2CVR = 0; TIM2->SWEVGR = 1u; Delay_Ms(200);
        TIM2->CTLR1 &= ~1u; TIM2->DMAINTENR |= (1u<<8);

        WS_SetPixel_RGB(i, 0, 0, 0); WS_encode(); WS_DMA_Start(ws_ccr, (uint16_t)(NUM_LEDS*24));
        TIM2->SWEVGR = 1u; TIM2->CTLR1 |= 1u;
        while (DMA_GetFlagStatus(DMA1_FLAG_TC2) == RESET) {}
        DMA_ClearFlag(DMA1_FLAG_TC2);
        DMA1_Channel2->CFGR &= ~1u; TIM2->DMAINTENR &= ~(1u<<8);
        TIM2->CH2CVR = 0; TIM2->SWEVGR = 1u; Delay_Ms(120);
        TIM2->CTLR1 &= ~1u; TIM2->DMAINTENR |= (1u<<8);
    }
}

static void WS_Show(void) {
    WS_encode();
    WS_DMA_Start(ws_ccr, (uint16_t)(NUM_LEDS*24));

    TIM2->SWEVGR = 1u;   /* UG */
    TIM2->CTLR1 |= 1u;   /* CEN */

    while (DMA_GetFlagStatus(DMA1_FLAG_TC2) == RESET) {}
    DMA_ClearFlag(DMA1_FLAG_TC2);

    DMA1_Channel2->CFGR &= ~1u;        /* EN=0 */
    TIM2->DMAINTENR     &= ~(1u<<8);   /* UDE=0 */
    TIM2->CH2CVR = 0;                  /* duty=0 → low */
    TIM2->SWEVGR = 1u;                 /* latch */
    Delay_Us(100);
    TIM2->CTLR1 &= ~1u;                /* stop */
    TIM2->DMAINTENR |= (1u<<8);        /* re-arm */
}

/* ============================================================
 *                   Parallel bus + framing
 *  Data: PC0..6 = D0..D6, PD0 = D7
 *  Clock: PD2 rising edge → EXTI7_0_IRQHandler
 * ============================================================ */
#define RX_MAX (3*NUM_LEDS + 8)

typedef enum { RX_SOF0, RX_SOF1, RX_ADDR, RX_CMD, RX_LEN, RX_PAY, RX_CRC } rx_state_t;
static volatile rx_state_t rx_state = RX_SOF0;

static volatile uint8_t  rx_addr=0, rx_cmd=0, rx_len=0, rx_crc=0;
static volatile uint8_t  rx_pay[RX_MAX];
static volatile uint16_t rx_idx=0;
static volatile uint8_t  frame_ready=0;

/* CRC8 Dallas/Maxim (reflected 0x31 → 0x8C) init 0x00 */
static uint8_t crc8_maxim(const uint8_t *p, size_t n){
    uint8_t crc = 0x00;
    while (n--){
        uint8_t in = *p++;
        for (uint8_t i=0;i<8;i++){
            uint8_t mix = (crc ^ in) & 0x01;
            crc >>= 1;
            if (mix) crc ^= 0x8C;
            in >>= 1;
        }
    }
    return crc;
}

/* Read D[7:0] : PC0..6→D0..6, PD0→D7 */
static inline uint8_t BUS_ReadByte(void){
    uint32_t c = GPIOC->INDR;
    uint32_t d = GPIOD->INDR;
    uint8_t v  = (uint8_t)(c & 0x7Fu);
    v |= (uint8_t)((d & 0x1u) << 7);
    return v;
}

/* Configure inputs + EXTI on PD2 */
static void ParallelBus_Init(void){
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_AFIO | RCC_PB2Periph_GPIOC | RCC_PB2Periph_GPIOD, ENABLE);

    GPIO_InitTypeDef io = {0};

    io.GPIO_Pin  = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |
                   GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
    io.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOC, &io);

    io.GPIO_Pin  = GPIO_Pin_0 | GPIO_Pin_2;
    io.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOD, &io);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource2);

    EXTI_InitTypeDef ex = {0};
    ex.EXTI_Line    = EXTI_Line2;
    ex.EXTI_Mode    = EXTI_Mode_Interrupt;
    ex.EXTI_Trigger = EXTI_Trigger_Rising;
    ex.EXTI_LineCmd = ENABLE;
    EXTI_Init(&ex);

    NVIC_InitTypeDef nv = {0};
    nv.NVIC_IRQChannel                   = EXTI7_0_IRQn; /* lines 0..7 share irq */
    nv.NVIC_IRQChannelPreemptionPriority = 1;
    nv.NVIC_IRQChannelSubPriority        = 1;
    nv.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nv);
}

/* EXTI lines 0..7 ISR (PD2 rising) */
void EXTI7_0_IRQHandler(void){
    if (EXTI_GetITStatus(EXTI_Line2) != RESET){
        uint8_t b = BUS_ReadByte();

        switch (rx_state){
        case RX_SOF0:
            rx_state = (b == SOF0) ? RX_SOF1 : RX_SOF0;
            break;
        case RX_SOF1:
            rx_state = (b == SOF1) ? RX_ADDR : RX_SOF0;
            break;
        case RX_ADDR:
            rx_addr = b; rx_state = RX_CMD; break;
        case RX_CMD:
            rx_cmd  = b; rx_state = RX_LEN; break;
        case RX_LEN:
            rx_len = b; rx_idx = 0;
            if (rx_len > (RX_MAX-8)) { rx_state = RX_SOF0; break; }
            rx_state = (rx_len ? RX_PAY : RX_CRC);
            break;
        case RX_PAY:
            rx_pay[rx_idx++] = b;
            if (rx_idx >= rx_len) rx_state = RX_CRC;
            break;
        case RX_CRC: {
            rx_crc = b;
            uint8_t hdr[3] = { rx_addr, rx_cmd, rx_len };
            uint8_t crc = crc8_maxim(hdr, 3);
            crc = crc8_maxim(rx_pay, rx_len) ^ crc;
            if (crc == rx_crc) frame_ready = 1;
            rx_state = RX_SOF0;
            } break;
        }

        EXTI_ClearITPendingBit(EXTI_Line2);
    }
}

/* Execute a validated frame in main context */
static void ExecFrame(void){
    if (!(rx_addr == ADDR_BCAST || rx_addr == DEVICE_ADDR)) {
        frame_ready = 0; return; /* not for us */
    }

    switch (rx_cmd){
    case CMD_SET_ALL_RGB:
        if (rx_len == 3*NUM_LEDS){
            for (size_t i=0, j=0; i<NUM_LEDS; i++, j+=3){
                WS_SetPixel_RGB(i, rx_pay[j+0], rx_pay[j+1], rx_pay[j+2]);
            }
            WS_Show();
        }
        break;

    case CMD_SET_ONE_RGB:
        if (rx_len == 4){
            uint8_t idx = rx_pay[0];
            if (idx < NUM_LEDS){
                WS_SetPixel_RGB(idx, rx_pay[1], rx_pay[2], rx_pay[3]);
                WS_Show();
            }
        }
        break;

    case CMD_FILL_RGB:
        if (rx_len == 3){
            for (size_t i=0; i<NUM_LEDS; i++)
                WS_SetPixel_RGB(i, rx_pay[0], rx_pay[1], rx_pay[2]);
            WS_Show();
        }
        break;

    case CMD_BOOT_TEST:
        WS_BootTest();
        break;

    default:
        /* ignore unknown */
        break;
    }

    frame_ready = 0;
}

/* ============================== main ============================== */
int main(void){
    SystemInit();
    Delay_Init();

    WS_LL_Init();        /* PC7 TIM2+DMA driver ready */
    ParallelBus_Init();  /* PD2 clock, PC0..6+PD0 data */

    WS_BootTest();       /* power-on self-test */

    while (1){
        if (frame_ready) ExecFrame();
        /* __WFI(); // optional low-power */
    }
}