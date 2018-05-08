/*
Copyright (c) 2012-2015 Ben Croston
Copyrigth (c) 2018 Esteban Volentini

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>
#include "c_gpio.h"
#include "event_gpio.h"

// GPIO really starts at offset SUNXI_LOW_GPIO_BASE + SUNXI_LOW_GPIO_OFFSET
#define SUNXI_LOW_GPIO_BASE  (0x01C20000)
#define SUNXI_LOW_GPIO_OFFSET 0x0800

// GPIO really starts at offset SUNXI_HIGH_GPIO_BASE + SUNXI_HIGH_GPIO_OFFSET
#define SUNXI_HIGH_GPIO_BASE  (0x01F02000)
#define SUNXI_HIGH_GPIO_OFFSET 0x0C00

#define LOW_GROUP     0
#define HIGH_GROUP    1

#define MAP_SIZE (1024)
#define MAP_MASK (MAP_SIZE - 1)

static const uint32_t gpio_offset[] = { SUNXI_LOW_GPIO_OFFSET, SUNXI_HIGH_GPIO_OFFSET };
static volatile uint32_t * gpio_map[2];

/* Nanopi mask pins available by banks */
static const int nanopi_PIN_MASK[9][32] = { // [BANK][INDEX]
    { 0,  1,  2,  3,  4,  5,  6,  7,  8,  9, -1, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,}, //PA
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,}, //PB
    { 0,  1,  2,  3, -1, -1, -1,  7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,}, //PC
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 14, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,}, //PD
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,}, //PE
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,}, //PF
    {-1, -1, -1, -1, -1, -1,  6,  7,  8,  9, -1, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,}, //PG
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,}, //PH

    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,}, //PL
};

/** Read a word from GPIO base + 'addr'. */
static uint32_t readl(uint8_t group, uint32_t addr) {
    uint32_t mmap_base = addr & MAP_MASK;
    uint32_t mmap_seek = (mmap_base + gpio_offset[group]);
    return gpio_map[group][mmap_seek >> 2];
}

/** Write a word to GPIO base + 'addr'. */
static void writel(uint8_t group, uint32_t val, uint32_t addr) {
    uint32_t mmap_base = addr & MAP_MASK;
    uint32_t mmap_seek = (mmap_base + gpio_offset[group]);
    gpio_map[group][mmap_seek >> 2] = val;
}

void short_wait(void) {
    int i;

    for (i=0; i<150; i++) {    // wait 150 cycles
        asm volatile("nop");
    }
}

int setup(void) {
    int mem_fd;
#if 0
    // try /dev/gpiomem first - this does not require root privs
    if ((mem_fd = open("/dev/gpiomem", O_RDWR|O_SYNC)) > 0) {
        gpio_map = (uint32_t *)mmap(NULL, MAP_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, 0);
        if (gpio_map == ((void*)-1)) {
            return SETUP_MMAP_FAIL;
        } else {
            return SETUP_OK;
        }
    }

#endif
    // revert to /dev/mem method - requires root
    if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0)
        return SETUP_DEVMEM_FAIL;

    // mmap the GPIO memory registers
    gpio_map[LOW_GROUP] = (uint32_t *) mmap (NULL, MAP_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, SUNXI_LOW_GPIO_BASE);
    if (gpio_map[LOW_GROUP] == ((void*) -1)) {
        return SETUP_MMAP_FAIL;
    }

    // mmap the GPIO memory registers
    gpio_map[HIGH_GROUP] = (uint32_t *) mmap (NULL, MAP_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, SUNXI_HIGH_GPIO_BASE);
    if (gpio_map[HIGH_GROUP] == ((void*) -1)) {
        munmap((void *)gpio_map[LOW_GROUP], MAP_SIZE);
        return SETUP_MMAP_FAIL;
    }

    return SETUP_OK;
}

void set_pullupdn(int gpio, int pud) {
    uint32_t regval = 0;
    uint8_t map = gpio >> 8;
    int bank = gpio >> 5;
    int index = gpio & 0x1F;
    int sub = index >> 4;
    int sub_index = index - 16 * sub;
    int offset = (bank & 0x07) * 0x24 + 0x1C + sub * 4; // +0x10 -> pullUpDn reg

    if (nanopi_PIN_MASK[bank][index] != -1) {
        regval = readl(map, offset);
        if(pud == PUD_OFF) {
            regval &= ~(3 << (sub_index << 1));
        } else if( pud == PUD_UP ) {
            regval &= ~(3 << (sub_index << 1));
            regval |= (1 << (sub_index << 1));
        } else if( pud == PUD_DOWN ) {
            regval &= ~(3 << (sub_index << 1));
            regval |= (2 << (sub_index << 1));
        } else {
            printf("set_pullupdn: pud number error %d\n", pud);
        }
        writel(map, regval, offset);
    } else {
        printf("set_pullupdn: pin number error %d\n", gpio);
    }
}

void setup_gpio(int gpio, int direction, int pud){
    int regval = 0;
    uint8_t map = gpio >> 8;
    int bank = gpio >> 5;
    int index = gpio & 0x1F;
    int offset = (bank & 0x07) * 0x24 + ((index >> 3) << 2);
    int val_offset = ((index - ((index >> 3) << 3)) << 2);

    set_pullupdn(gpio, pud);

    if (nanopi_PIN_MASK[bank][index] != -1) {
        regval = readl(map, offset);
        if(direction == OUTPUT) {
            regval &= ~(7 << val_offset);
            regval |= (1 << val_offset);
        } else if( direction == INPUT ) {
            regval &= ~(7 << val_offset);
        } else {
            printf("setup_gpio: direction number error %d\n", direction);
        }
        writel(map, regval, offset);
    } else {
        printf("setup_gpio: pin number error %d\n", gpio);
    }
}


// Contribution by Eric Ptak <trouch@trouch.com>
int gpio_function(int gpio) {
    int regval = 0;
    uint8_t map = gpio >> 8;
    int bank = gpio >> 5;
    int index = gpio & 0x1F ;
    int offset = (bank & 0x07) * 0x24 + ((index >> 3) << 2);
    int val_offset = ((index - ((index >> 3) << 3)) << 2);

    if (nanopi_PIN_MASK[bank][index] != -1) {
        regval = (readl(map, offset) >> val_offset) & 0x07;
    } else {
        printf("gpio_function: pin number error %d\n", gpio);
    }
    return regval;
}

void output_gpio(int gpio, int value) {
    int regval = 0;
    uint8_t map = gpio >> 8;
    int bank = gpio >> 5;
    int offset = (bank & 0x07) * 0x24 + 0x10; // +0x10 -> data reg
    int index = gpio & 0x1F ;
    int mask = 1 << index;

    if (nanopi_PIN_MASK[bank][index] != -1) {
        regval = readl(map, offset);
        regval = value ? (regval | mask) : (regval & ~mask);
        writel(map, regval, offset);
    } else {
        printf("output_gpio: pin number error %d\n", gpio);
    }
}

int input_gpio(int gpio) {
    struct gpios* g = get_gpio(gpio);
    int regval = 0;

    if(g == NULL) {
      /* g is NULL, consequently gpio has not been registered for interrupts,
         we can do a direct memory access */
      uint8_t map = gpio >> 8;
      int bank = gpio >> 5;
      int offset = (bank & 0x07)* 0x24 + 0x10; // +0x10 -> data reg
      int index = gpio & 0x1F ;
      int mask = 1 << index;

      if (nanopi_PIN_MASK[bank][index] != -1) {
          regval = readl(map, offset);
          regval = regval & mask;
      } else {
          printf("input_gpio: pin number error %d\n", gpio);
      }
    } else {
      /* On interrupt muxing, we need first to remux to value mode. However,
         we prefer to leave this work to the kernel instead. Consequently,
         we read the value through the /sys/class/gpio/../value. */
      char buffer = 0;

      /* Return to start position */
      lseek(g->value_fd, 0, SEEK_SET);

      if (read(g->value_fd, &buffer, sizeof(buffer)) != sizeof(buffer)) {
         perror("input_gpio read");
      }

      regval = buffer == '0' ? 0 : 1;
    }

    return regval;
}

void cleanup(void) {
    munmap((void *)gpio_map[LOW_GROUP], MAP_SIZE);
    munmap((void *)gpio_map[HIGH_GROUP], MAP_SIZE);
}
