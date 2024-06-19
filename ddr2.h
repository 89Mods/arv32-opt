#ifndef DDR2_H_
#define DDR2_H_

void ddr2_init(void);
void ddr2_read(uint32_t addr, uint8_t* vals, uint8_t count);
void ddr2_write(uint32_t addr, uint8_t* val, uint8_t count);

#endif
