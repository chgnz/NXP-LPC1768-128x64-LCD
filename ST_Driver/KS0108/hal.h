/*PORT 0*/
#include <stdint.h>
#include <stdbool.h>

#include "common.h"

#define KS0108_RS_H   output_high(3, 26)
#define KS0108_RS_L   output_low(3, 26)
#define KS0108_RW_H   output_high(3, 25)
#define KS0108_RW_L   output_low(3, 25)
#define KS0108_E_H    output_high(0, 27)
#define KS0108_E_L    output_low(0, 27)
#define KS0108_CS1_H  output_high(1, 18)
#define KS0108_CS1_L  output_low(1, 18)
#define KS0108_CS2_H  output_high(0, 30)
#define KS0108_CS2_L  output_low(0, 30)
#define KS0108_RST_H  output_high(0, 29)
#define KS0108_RST_L  output_low(0, 29)



void KS0108_WRITE_DATA(uint8_t data);
uint8_t KS0108_READ_DATA();

