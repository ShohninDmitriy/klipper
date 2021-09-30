// Hardware PWM support on stm32
//
// Copyright (C) 2021  Michael Kurz <michi.kurz@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "board/irq.h" // irq_save
#include "command.h" // shutdown
#include "gpio.h" // gpio_pwm_write
#include "internal.h" // GPIO
#include "sched.h" // sched_shutdown

#define MAX_PWM 255
DECL_CONSTANT("PWM_MAX", MAX_PWM);

struct gpio_pwm_info {
    TIM_TypeDef* timer;
    uint8_t pin, channel, function;
};

static const struct gpio_pwm_info pwm_regs[] = {
//  {timer, pin,    channel, function}
#if CONFIG_MACH_STM32F103
    {TIM2, GPIO('A', 0),  1, GPIO_FUNCTION(2)},
    {TIM2, GPIO('A', 1),  2, GPIO_FUNCTION(2)},
    {TIM2, GPIO('A', 2),  3, GPIO_FUNCTION(2)},
    {TIM2, GPIO('A', 3),  4, GPIO_FUNCTION(2)},
    {TIM2, GPIO('A', 15), 1, GPIO_FUNCTION(1)},
    {TIM2, GPIO('B', 3),  2, GPIO_FUNCTION(1)},
    {TIM2, GPIO('B', 10), 3, GPIO_FUNCTION(1)},
    {TIM2, GPIO('B', 11), 4, GPIO_FUNCTION(1)},
    {TIM3, GPIO('A', 6),  1, GPIO_FUNCTION(1)},
    {TIM3, GPIO('A', 7),  2, GPIO_FUNCTION(1)},
    {TIM3, GPIO('B', 0),  3, GPIO_FUNCTION(1)},
    {TIM3, GPIO('B', 1),  4, GPIO_FUNCTION(1)},
    {TIM3, GPIO('C', 6),  1, GPIO_FUNCTION(2)},
    {TIM3, GPIO('C', 7),  2, GPIO_FUNCTION(2)},
    {TIM3, GPIO('C', 8),  3, GPIO_FUNCTION(2)},
    {TIM3, GPIO('C', 9),  4, GPIO_FUNCTION(2)},
    {TIM4, GPIO('D', 12), 1, GPIO_FUNCTION(2)},
    {TIM4, GPIO('D', 13), 2, GPIO_FUNCTION(2)},
    {TIM4, GPIO('D', 14), 3, GPIO_FUNCTION(2)},
    {TIM4, GPIO('D', 15), 4, GPIO_FUNCTION(2)},
    {TIM4, GPIO('B', 6),  1, GPIO_FUNCTION(2)},
    {TIM4, GPIO('B', 7),  2, GPIO_FUNCTION(2)},
    {TIM4, GPIO('B', 8),  3, GPIO_FUNCTION(2)},
    {TIM4, GPIO('B', 9),  4, GPIO_FUNCTION(2)}
#elif CONFIG_MACH_STM32F446 //octopus
//    {TIM1,  GPIO('A',  8),  1, GPIO_FUNCTION(1)}, // FAN0             Tested - Not Working
//    {TIM1,  GPIO('E',  9),  1, GPIO_FUNCTION(1)}, // EXP1_3           Tested - Not Working
    {TIM2,  GPIO('A',  5),  1, GPIO_FUNCTION(1)}, //   EXP2_2           Tested - Working
//    {TIM2,  GPIO('A', 15),  1, GPIO_FUNCTION(1)}, // SPI3             Tested - Working
//    {TIM2,  GPIO('B',  3),  2, GPIO_FUNCTION(1)}, // SPI3
    {TIM2,  GPIO('B', 10),  3, GPIO_FUNCTION(1)}, //   HE2              Tested - Working
    {TIM2,  GPIO('B',  2),  4, GPIO_FUNCTION(1)}, //   EXP2_5
    {TIM2,  GPIO('B', 11),  4, GPIO_FUNCTION(1)}, //   HE3              Tested - Working
//    {TIM3,  GPIO('B',  4),  1, GPIO_FUNCTION(2)}, // SPI3             Tested - Working
//    {TIM3,  GPIO('A',  7),  2, GPIO_FUNCTION(2)}, // EXP2_6           Tested - Working ?????
//    {TIM3,  GPIO('B',  5),  2, GPIO_FUNCTION(2)}, // SPI3             Tested - Working
    {TIM3,  GPIO('B',  0),  3, GPIO_FUNCTION(2)}, //   RGB_LED
    {TIM3,  GPIO('B',  1),  4, GPIO_FUNCTION(2)}, //   EXP2_3
    {TIM4,  GPIO('B',  6),  1, GPIO_FUNCTION(2)}, //   PWM_BL_TOUCH
    {TIM4,  GPIO('D', 12),  1, GPIO_FUNCTION(2)}, //   FAN2             Tested - Working
//    {TIM4,  GPIO('B',  7),  2, GPIO_FUNCTION(2)}, // END_BL_TOUCH
    {TIM4,  GPIO('D', 13),  2, GPIO_FUNCTION(2)}, //   FAN3             Tested - Working
//    {TIM4,  GPIO('D', 14),  3, GPIO_FUNCTION(2)}, // FAN4             Tested - Not Working crash mcu
//    {TIM4,  GPIO('D', 15),  4, GPIO_FUNCTION(2)}, // FAN5             Tested - Not Working crash mcu
//    {TIM5,  GPIO('A',  0),  1, GPIO_FUNCTION(2)}, // EN_DRIVER3
    {TIM5,  GPIO('A',  1),  2, GPIO_FUNCTION(2)}, //   HBED             Tested - Working
    {TIM5,  GPIO('A',  2),  3, GPIO_FUNCTION(2)}, //   HE0              Tested - Working
    {TIM5,  GPIO('A',  3),  4, GPIO_FUNCTION(2)}, //   HE1              Tested - Working
//    {TIM8,  GPIO('C',  6),  1, GPIO_FUNCTION(3)}, // CS_DRIVER2
//    {TIM8,  GPIO('C',  7),  2, GPIO_FUNCTION(3)}, // CS_DRIVER3
//    {TIM8,  GPIO('C',  8),  3, GPIO_FUNCTION(3)}, // sdio
//    {TIM8,  GPIO('C',  9),  4, GPIO_FUNCTION(3)}, // sdio
    {TIM9,  GPIO('E',  5),  1, GPIO_FUNCTION(3)}, //   FAN1             Tested - Working
//    {TIM9,  GPIO('E',  6),  2, GPIO_FUNCTION(3)}, // STEP_DRIVER7
    {TIM10, GPIO('B',  8),  1, GPIO_FUNCTION(3)}, //   I2C_SCL
//    {TIM10, GPIO('F',  6),  1, GPIO_FUNCTION(3)}, // T2
    {TIM11, GPIO('B',  9),  1, GPIO_FUNCTION(3)}, //   I2C_SDA
//    {TIM11, GPIO('F',  7),  1, GPIO_FUNCTION(3)}, // T3
//    {TIM12, GPIO('B', 14),  1, GPIO_FUNCTION(9)}, // USB
//    {TIM12, GPIO('B', 15),  2, GPIO_FUNCTION(9)}, // USB
    {TIM13, GPIO('A',  6),  1, GPIO_FUNCTION(9)}, //   EXP2_1           Tested - Working
//    {TIM13, GPIO('F',  8),  1, GPIO_FUNCTION(9)}, // PT100
    {TIM14, GPIO('A',  7),  1, GPIO_FUNCTION(9)} //    EXP2_6           Tested - Working ?????
//    {TIM14, GPIO('F',  9),  1, GPIO_FUNCTION(9)} //  STEP_DRIVER4
#endif
};

struct gpio_pwm
gpio_pwm_setup(uint8_t pin, uint32_t cycle_time, uint8_t val){
    // Find pin in pwm_regs table
    const struct gpio_pwm_info* p = pwm_regs;
    for (;; p++) {
        if (p >= &pwm_regs[ARRAY_SIZE(pwm_regs)])
            shutdown("Not a valid PWM pin");
        if (p->pin == pin)
            break;
    }

    // Map cycle_time to pwm clock divisor
    uint32_t pclk = get_pclock_frequency((uint32_t)p->timer);
    uint32_t pclock_div = CONFIG_CLOCK_FREQ / pclk;
    if (pclock_div > 1)
        pclock_div /= 2; // Timers run at twice the normal pclock frequency
    uint32_t prescaler = cycle_time / (pclock_div * (MAX_PWM - 1));
    if (prescaler > 0) {
        prescaler -= 1;
    } else if (prescaler > UINT16_MAX) {
        prescaler = UINT16_MAX;
    }

    gpio_peripheral(p->pin, p->function, 0);

    // Enable clock
    if (!is_enabled_pclock((uint32_t) p->timer)) {
        enable_pclock((uint32_t) p->timer);
    }

    if (p->timer->CR1 & TIM_CR1_CEN) {
        if (p->timer->PSC != (uint16_t) prescaler) {
            shutdown("PWM already programmed at different speed");
        }
    } else {
        p->timer->PSC = (uint16_t) prescaler;
        p->timer->ARR = MAX_PWM - 1;
        p->timer->EGR |= TIM_EGR_UG;
    }

    struct gpio_pwm channel;
    switch (p->channel) {
        case 1: {
            channel.reg = (void*) &p->timer->CCR1;
            p->timer->CCER &= ~TIM_CCER_CC1E;
            p->timer->CCMR1 &= ~(TIM_CCMR1_OC1M | TIM_CCMR1_CC1S);
            p->timer->CCMR1 |= (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 |
                                TIM_CCMR1_OC1PE | TIM_CCMR1_OC1FE);
            gpio_pwm_write(channel, val);
            p->timer->CCER |= TIM_CCER_CC1E;
            break;
        }
        case 2: {
            channel.reg = (void*) &p->timer->CCR2;
            p->timer->CCER &= ~TIM_CCER_CC2E;
            p->timer->CCMR1 &= ~(TIM_CCMR1_OC2M | TIM_CCMR1_CC2S);
            p->timer->CCMR1 |= (TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 |
                                TIM_CCMR1_OC2PE | TIM_CCMR1_OC2FE);
            gpio_pwm_write(channel, val);
            p->timer->CCER |= TIM_CCER_CC2E;
            break;
        }
        case 3: {
            channel.reg = (void*) &p->timer->CCR3;
            p->timer->CCER &= ~TIM_CCER_CC3E;
            p->timer->CCMR2 &= ~(TIM_CCMR2_OC3M | TIM_CCMR2_CC3S);
            p->timer->CCMR2 |= (TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 |
                                TIM_CCMR2_OC3PE | TIM_CCMR2_OC3FE);
            gpio_pwm_write(channel, val);
            p->timer->CCER |= TIM_CCER_CC3E;
            break;
        }
        case 4: {
            channel.reg = (void*) &p->timer->CCR4;
            p->timer->CCER &= ~TIM_CCER_CC4E;
            p->timer->CCMR2 &= ~(TIM_CCMR2_OC4M | TIM_CCMR2_CC4S);
            p->timer->CCMR2 |= (TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 |
                                TIM_CCMR2_OC4PE | TIM_CCMR2_OC4FE);
            gpio_pwm_write(channel, val);
            p->timer->CCER |= TIM_CCER_CC4E;
            break;
        }
        default:
            shutdown("Invalid PWM channel");
    }
    // Enable PWM output
    p->timer->CR1 |= TIM_CR1_CEN;

    return channel;
}

void
gpio_pwm_write(struct gpio_pwm g, uint32_t val) {
    *(volatile uint32_t*) g.reg = val;
}
