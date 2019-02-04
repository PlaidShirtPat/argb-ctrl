/*
 * SPI testing utility (using spidev driver)
 *
 * Copyright (c) 2007  MontaVista Software, Inc.
 * Copyright (c) 2007  Anton Vorontsov <avorontsov@ru.mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * Cross-compile with cross-gcc -I/path/to/cross-kernel/include
 */

#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

struct argb_spi_color {
  uint8_t*  data;
  size_t    data_len;
  uint32_t  speed;
};

static void pabort(const char *s)
{
  perror(s);
  abort();
}

#define F125_MHZ 125000000
#define F7800_KHZ 7800000
static const char *device = "/dev/spidev0.0";
static uint8_t mode = SPI_CPHA;
static uint8_t bits = 8;
static uint32_t speed = F7800_KHZ;
static uint16_t delay = 0;
static uint8_t arg_red = 0;
static uint8_t arg_green = 0;
static uint8_t arg_blue = 0;


static void transfer(int fd, struct argb_spi_color* argb)
{
#define N_BUFS 10
  int ret;
  size_t size = 2000;
  //uint8_t *tx = calloc(size*N_BUFS, sizeof(uint8_t));
  //uint8_t *rx = calloc(size*N_BUFS, sizeof(uint8_t));
  /* uint8_t tx[] = { */
  /*   //reset */
  /*   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, */
  /*   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, */
  /*   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, */
  /*   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, */
  /*   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, */
  /*   //first color 255, 0, 255, */
  /*   //0b11111100, 0b00011111, 0b10000011, 0b11110000, 0b01111110, 0b00001111 */
  /*   0xfc, 0x1f, 0x83, 0xf0, 0x7e, 0x0f, */
  /*   0xfc, 0x1f, 0x83, 0xf0, 0x7e, 0x0f, */
  /*   0xfc, 0x1f, 0x83, 0xf0, 0x7e, 0x0f, */
  /*   0xfc, 0x1f, 0x83, 0xf0, 0x7e, 0x0f, */
  /*   0xfc, 0x1f, 0x83, 0xf0, 0x7e, 0x0f, */
  /*   0xfc, 0x1f, 0x83, 0xf0, 0x7e, 0x0f, */
  /*   0xfc, 0x1f, 0x83, 0xf0, 0x7e, 0x0f, */
  /*   0xfc, 0x1f, 0x83, 0xf0, 0x7e, 0x0f, */
  /*   0xfc, 0x1f, 0x83, 0xf0, 0x7e, 0x0f, */
  /*   0xfc, 0x1f, 0x83, 0xf0, 0x7e, 0x0f, */
  /*   0xfc, 0x1f, 0x83, 0xf0, 0x7e, 0x0f, */
  /*   0xfc, 0x1f, 0x83, 0xf0, 0x7e, 0x0f, */
  /*   0xfc, 0x1f, 0x83, 0xf0, 0x7e, 0x0f, */
  /*   0xfc, 0x1f, 0x83, 0xf0, 0x7e, 0x0f, */
  /*   0xfc, 0x1f, 0x83, 0xf0, 0x7e, 0x0f, */
  /*   0xfc, 0x1f, 0x83, 0xf0, 0x7e, 0x0f, */
  /*   0xfc, 0x1f, 0x83, 0xf0, 0x7e, 0x0f, */
  /*   0xfc, 0x1f, 0x83, 0xf0, 0x7e, 0x0f, */
  /*   0xfc, 0x1f, 0x83, 0xf0, 0x7e, 0x0f, */
  /*   0xfc, 0x1f, 0x83, 0xf0, 0x7e, 0x0f, */
  /*   0xfc, 0x1f, 0x83, 0xf0, 0x7e, 0x0f, */
  /*   0xfc, 0x1f, 0x83, 0xf0, 0x7e, 0x0f, */
  /*   0xfc, 0x1f, 0x83, 0xf0, 0x7e, 0x0f, */
  /*   0xfc, 0x1f, 0x83, 0xf0, 0x7e, 0x0f, */
  /*   0xfc, 0x1f, 0x83, 0xf0, 0x7e, 0x0f, */
  /*   0xfc, 0x1f, 0x83, 0xf0, 0x7e, 0x0f, */
  /*   0xfc, 0x1f, 0x83, 0xf0, 0x7e, 0x0f, */
  /*   0xfc, 0x1f, 0x83, 0xf0, 0x7e, 0x0f, */
  /*   0xfc, 0x1f, 0x83, 0xf0, 0x7e, 0x0f, */
  /*   0xfc, 0x1f, 0x83, 0xf0, 0x7e, 0x0f, */
  /*   0xfc, 0x1f, 0x83, 0xf0, 0x7e, 0x0f, */
  /*   0xfc, 0x1f, 0x83, 0xf0, 0x7e, 0x0f, */
  /*   0xfc, 0x1f, 0x83, 0xf0, 0x7e, 0x0f, */
  /*   0xfc, 0x1f, 0x83, 0xf0, 0x7e, 0x0f, */
  /*   0xfc, 0x1f, 0x83, 0xf0, 0x7e, 0x0f, */
  /*   0xfc, 0x1f, 0x83, 0xf0, 0x7e, 0x0f, */
  /*   0xfc, 0x1f, 0x83, 0xf0, 0x7e, 0x0f, */
  /*   0xfc, 0x1f, 0x83, 0xf0, 0x7e, 0x0f, */
  /*   0xfc, 0x1f, 0x83, 0xf0, 0x7e, 0x0f, */
  /*   0xfc, 0x1f, 0x83, 0xf0, 0x7e, 0x0f, */
  /*   0xfc, 0x1f, 0x83, 0xf0, 0x7e, 0x0f, */
  /*   0xfc, 0x1f, 0x83, 0xf0, 0x7e, 0x0f, */
  /*   0xfc, 0x1f, 0x83, 0xf0, 0x7e, 0x0f, */
  /*   0xfc, 0x1f, 0x83, 0xf0, 0x7e, 0x0f, */
  /*   0xfc, 0x1f, 0x83, 0xf0, 0x7e, 0x0f, */
  /*   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, */
  /*   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, */
  /*   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, */
  /*   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, */
  /*   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, */

  /* }; */


  struct spi_ioc_transfer tr = {
    .delay_usecs = delay,
    .bits_per_word = bits,
    .tx_buf = (unsigned long)argb->data,
    .rx_buf = (unsigned long)NULL,
    .len = argb->data_len,
    .speed_hz = argb->speed
  };

  ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
  if (ret < 1)
    pabort("can't send spi message");



  for(int i=0; i < argb->data_len; i++) {
    uint8_t num = argb->data[i];
    for(int j=0; j < 8; j++){
        // print last bit and shift left.
        printf("%u", num&128 ? 1 : 0);
        num = num<<1;
    }
  }

  return;


}

static void print_usage(const char *prog)
{
  printf("Usage: %s [-DsbdlHOLC3]\n", prog);
  puts("  -D --device   device to use (default /dev/spidev1.1)\n"
      "  -s --speed    max speed (Hz)\n"
      "  -d --delay    delay (usec)\n"
      "  -b --bpw      bits per word \n"
      "  -l --loop     loopback\n"
      "  -H --cpha     clock phase\n"
      "  -O --cpol     clock polarity\n"
      "  -L --lsb      least significant bit first\n"
      "  -C --cs-high  chip select active high\n"
      "  -3 --3wire    SI/SO signals shared\n");
  exit(1);
}

static void parse_opts(int argc, char *argv[])
{
  while (1) {
    static const struct option lopts[] = {
      { "device",  1, 0, 'D' },
      { "speed",   1, 0, 's' },
      { "red",     1, 0, 'r' },
      { "green",   1, 0, 'g' },
      { "blue",    1, 0, 'b' },
      { "delay",   1, 0, 'd' },
      { "loop",    0, 0, 'l' },
      { "cpha",    0, 0, 'H' },
      { "cpol",    0, 0, 'O' },
      { "lsb",     0, 0, 'L' },
      { "cs-high", 0, 0, 'C' },
      { "3wire",   0, 0, '3' },
      { "no-cs",   0, 0, 'N' },
      { "ready",   0, 0, 'R' },
      { NULL, 0, 0, 0 },
    };
    int c;

    c = getopt_long(argc, argv, "r:g:b:D:s:d:lHOLC3NR", lopts, NULL);

    if (c == -1)
      break;

    switch (c) {
      case 'D':
        device = optarg;
        break;
      case 's':
        speed = atoi(optarg);
        break;
      case 'd':
        delay = atoi(optarg);
        break;
      case 'r':
        arg_red = atoi(optarg);
        break;
      case 'g':
        arg_green = atoi(optarg);
        break;
      case 'b':
        arg_blue = atoi(optarg);
        break;
      case 'l':
        mode |= SPI_LOOP;
        break;
      case 'H':
        mode |= SPI_CPHA;
        break;
      case 'O':
        mode |= SPI_CPOL;
        break;
      case 'L':
        mode |= SPI_LSB_FIRST;
        break;
      case 'C':
        mode |= SPI_CS_HIGH;
        break;
      case '3':
        mode |= SPI_3WIRE;
        break;
      case 'N':
        mode |= SPI_NO_CS;
        break;
      case 'R':
        mode |= SPI_READY;
        break;
      default:
        print_usage(argv[0]);
        break;
    }
  }
}

// even bits are high bits, odd bits are low bits
bool is_high_bit(int nbit) {
  return (nbit % 2) == 0;
}

#define ARGB_H0 3
#define ARGB_H1 5
#define ARGB_L0 6
#define ARGB_L1 4
// write encoded bits to data starting at nbit, then update nbit
void encode_argb_bit(bool bit, uint8_t* data, size_t* nbit, bool is_high) {
  int currbyte = *nbit / 8;
  int currbit  = *nbit % 8;
  // address by 16bit word since a bit will span at most 2 bytes
  uint16_t tmp = data[currbyte] << 8;
  int hbit_len  = (bit ? ARGB_H1 : ARGB_H0);
  int lbit_len  = (bit ? ARGB_L1 : ARGB_L0);
  // if(is_high)
  //   bit_len = (bit ? ARGB_H1 : ARGB_H0);
  // else
  //   bit_len = (bit ? ARGB_L1 : ARGB_L0);

  // bit toggling logic found here: https://stackoverflow.com/questions/47981/how-do-you-set-clear-and-toggle-a-single-bit
  // we write bits from high -> low
  for(int i=0; i < hbit_len; i++) {
      tmp |= 1 << (15 - (currbit + i));
  }
  for(int i=hbit_len; i < hbit_len+lbit_len; i++) {
      tmp &= ~(1 << (15 - (currbit + i)));
  }

  // update data
  data[currbyte] = tmp >> 8;
  data[currbyte+1] = (uint8_t)tmp;
  // update len
  *nbit = *nbit + hbit_len + lbit_len;
}

// each bit takes a high and a low bit, we include extra room
#define GRB_BITS 156*2
// technically 391, but giving a bit of extra room
#define RESET_BITS 400
struct argb_spi_color* create_argb_spi_color(uint8_t r, uint8_t g, uint8_t b, size_t num_leds) {
  struct argb_spi_color* argb = malloc(sizeof(*argb));
  // len = total_bits / 8 + 1 (+1 since integer division rounds down)
  size_t bytes_len = ((RESET_BITS*2 + (GRB_BITS * num_leds)) / 8) + 2;
  uint8_t* bytes = calloc(sizeof(*bytes), bytes_len);
  argb->data = bytes;
  argb->data_len = bytes_len;
  argb->speed = F7800_KHZ;

  //now we go bit by bit and encode them
  // start at RESET_BITS to leave first bit low for reset
  size_t nbits = RESET_BITS;
  bool is_high = true;
  // encode GRB values for each LED
  for(int i=0; i < num_leds; i++) {
    // encode green
    for(int j=0; j < 8; j++) {
      encode_argb_bit((g >> (7-j)) & 1U, bytes, &nbits, is_high);
      is_high = !is_high;
    }
    // encode red
    for(int j=0; j < 8; j++) {
      encode_argb_bit((r >> (7-j)) & 1U, bytes, &nbits, is_high);
      is_high = !is_high;
    }
    // encode blue
    for(int j=0; j < 8; j++) {
      encode_argb_bit((b >> (7-j)) & 1U, bytes, &nbits, is_high);
      is_high = !is_high;
    }
  }

  return argb;
}

void free_argb_spi_color(struct argb_spi_color* argb) {
  free(argb);
}

int main(int argc, char *argv[])
{
  int ret = 0;
  int fd;

  parse_opts(argc, argv);

  fd = open(device, O_RDWR);
  if (fd < 0)
    pabort("can't open device");

  /*
   * spi mode
   */
  ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
  if (ret == -1)
    pabort("can't set spi mode");

  ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
  if (ret == -1)
    pabort("can't get spi mode");

  /*
   * bits per word
   */
  ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
  if (ret == -1)
    pabort("can't set bits per word");

  ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
  if (ret == -1)
    pabort("can't get bits per word");

  /*
   * max speed hz
   */
  ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
  if (ret == -1)
    pabort("can't set max speed hz");

  ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
  if (ret == -1)
    pabort("can't get max speed hz");

  printf("spi mode: %d\n", mode);
  printf("bits per word: %d\n", bits);
  printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);
  
  struct argb_spi_color* argb = create_argb_spi_color(arg_red, arg_green, arg_blue, 8);

  transfer(fd, argb);

  close(fd);

  return ret;
}
