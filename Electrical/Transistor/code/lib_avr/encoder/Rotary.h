#ifndef _LIB_AVR_ROTARYENCODER_H_
#define _LIB_AVR_ROTARYENCODER_H_

#include "Encoder.h"


/** @brief allow quadrature decoding with a single input by defining
 *
 *  Several quadrature encoder states could be reached by missing ticks (if
 *  interrupts are enabled on both edges) or by normal operation if interrupts
 *  only come from one edge. We assume one edge interrupts in this case. Some
 *  encoders that need debouncing work better by not counting these state. You
 *  should comment this define in that case.
 *
 *  Note that single int quadrature decoding did not work well in testing, but
 *  it is included in the same form in other libraries.
 */
#define ENCODER_ALLOW_SINGLE_INT

class Rotary : public Encoder {
    uint8_t config_;
    volatile uint8_t *pin_b_reg_;
    uint8_t pin_b_num_;
    volatile long ticks_;
    volatile uint8_t pin_state_last_;
    volatile uint8_t errors_;

    public:
        //Note: the super class's constructor is also called
        Rotary();

        /** @brief initialize a single wire encoder */
        uint8_t Init(volatile uint8_t *pin_a_reg,
                     uint8_t pin_a_num,
                     volatile uint8_t *port_addr,
                     volatile uint8_t *ddr_addr);
        /** @brief to be called on interrupt of single wire encoder signal */
        //void OnInterrupt();
        void OnInterrupt() override;
};

#endif /* _LIB_AVR_ROTARYENCODER_H_ */

