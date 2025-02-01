#ifndef CRC_H_
#define CRC_H

#include <inttypes.h>

uint16_t getCRC(uint16_t crc_accum, uint8_t* data_blk_ptr,
                uint16_t data_blk_size);

#endif  // CRC_H_