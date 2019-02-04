# Addresable RGB Controller

## Discovery

```
Sample bits
taken @ approx 7.8 MHz
roughly 1.28E-7 seconds per bit
0.128 usec resolution
reset is after >=50 usec
reset takes 391 bits (48B) (390.625 rounded up)
1110000001110000001111110001110000001111110001111100001100000001
1000000000001110000001110000001100000001100000001100000001100000
0111000000111000000111000000111000000111111000111000000111110000
1111100001100000011100000011100000011100000011111100011100000011
1111000111110000110000000110000000000011100000011100000011100000
0110000000110000000110000001110000001110000001110000001110000001
1111100011100000011111000011111000011000000111000000111000000111
0000001111110001110000001111110001111110001110000001100000000000
1110000001110000001110000001100000001100000011100000011100000011
1000000111000000111000000111111000111000000111111000111110000110
0000011100000011100000011100000011111100011100000011111100011111
1000111000000111100000000011100000011100000011100000011100000011

111 - approx .38 us, a 0H
0000 - approx .51 us,  1L
0000000 - approx .89 us, a 0L
111111 -approx .77 us, a 1H
3-4 chars are likely the short, 6+ are likely the long



Optimal for write is likely:
NOTE: inverted from read (using the voltage amplifier)
0H: 000      ~0.38 usec (0.35 usec) within tolerance (+/- 0.15 usec)
1H: 000000   ~0.77 usec (0.70 usec) within tolerance (+/- 0.15 usec)
0L: 1111111  ~0.90 usec (0.80 usec) within tolerance (+/- 0.15 usec)
1L: 11111    ~0.64 usec (0.60 usec) within tolerance (+/- 0.15 usec)
```



## Raspi Spi Limitations

Can only handle speeds that are are some divsion of 2^n of the main clock speed

From [raspberry pi website](https://www.raspberrypi.org/documentation/hardware/raspberrypi/spi/README.md):
```
  cdiv    speed
     2    125.0 MHz
     4     62.5 MHz
     8     31.2 MHz
    16     15.6 MHz
    32      7.8 MHz
    64      3.9 MHz
   128     1953 kHz
   256      976 kHz
   512      488 kHz
  1024      244 kHz
  2048      122 kHz
  4096       61 kHz
  8192     30.5 kHz
 16384     15.2 kHz
 32768     7629 Hz
```


- We use 7.8MHz speed to get within the timing restrictions of the WS2812 protocol
- Also, transfers must be greater than 96 bytes (source: I can't find the article I read, but it seems to work?)
