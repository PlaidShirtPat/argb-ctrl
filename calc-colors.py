#!/usr/bin/env python3
data0 = [
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
0xFF, 0xFF, 0xE3, 0x81, 0xF8, 0xE0, 0x7E, 0x3F, 
0x1C, 0x0E, 0x07, 0xC7, 0x03, 0xF1, 0xC0, 0xFC, 
0x7E, 0x38, 0x1C, 0x00, 0xE0, 0x70, 0x38, 0x1C, 
0x0E, 0x07, 0x03, 0x83, 0x81, 0xF8, 0xE0, 0x7E, 
0x38, 0x1F, 0x8F, 0xC7, 0x03, 0x81, 0xF1, 0xC0, 
0xFC, 0x70, 0x3F, 0x1F, 0x8E, 0x07, 0x00, 0x38, 
0x1C, 0x0E, 0x07, 0x03, 0x81, 0xC0, 0xE0, 0x60, 
0x7E, 0x38, 0x1F, 0x8E, 0x07, 0xE3, 0xF1, 0xC0, 
0xE0, 0x7C, 0x30, 0x3F, 0x1C, 0x0F, 0xC7, 0xE3, 
0x81, 0xC0, 0x0E, 0x07, 0x03, 0x81, 0xC0, 0xE0, 
0x70, 0x38, 0x18, 0x1F, 0x8E, 0x07, 0xE3, 0x81, 
0xF8, 0xFC, 0x70, 0x38, 0x1F, 0x0C, 0x0F, 0xC7, 
0x03, 0xF1, 0xF8, 0xE0, 0x70, 0x03, 0x81, 0xC0, 
0xE0, 0x70, 0x38, 0x1C, 0x0E, 0x06, 0x03, 0xE3, 
0x81, 0xF8, 0xE0, 0x7E, 0x3F, 0x1C, 0x0E, 0x07, 
0xE3, 0x81, 0xF1, 0xC0, 0xFC, 0x7E, 0x38, 0x1C, 
0x00, 0xE0, 0x70, 0x38, 0x1C, 0x0E, 0x07, 0x03, 
0x81, 0xC0, 0xF8, 0xE0, 0x7E, 0x38, 0x1F, 0x8F, 
0xC7, 0x03, 0x81, 0xF8, 0xE0, 0x7C, 0x70, 0x3F, 
0x1F, 0x8E, 0x07, 0x00, 0x38, 0x1C, 0x0E, 0x07, 
0x03, 0x81, 0xC0, 0xE0, 0x70, 0x3E, 0x18, 0x1F, 
0x8E, 0x07, 0xE3, 0xF1, 0xC0, 0xE0, 0x7E, 0x38, 
0x1F, 0x0C, 0x0F, 0xC7, 0xE3, 0x81, 0xC0, 0x0E, 
0x07, 0x03, 0x81, 0xC0, 0xE0, 0x70, 0x38, 0x1C, 
0x0F, 0x86, 0x07, 0xE3, 0x81, 0xF8, 0xFC, 0x70, 
0x38, 0x1F, 0x8E, 0x07, 0xC3, 0x03, 0xF1, 0xF8, 
0xE0, 0x70, 0x01, 0x81, 0xC0, 0xE0, 0x70, 0x38, 
0x1C, 0x0E, 0x07, 0x03, 0xE1, 0x80, 0xF8, 0xE0, 
0x7E, 0x3F, 0x1C, 0x0E, 0x07, 0xE3, 0x81, 0xF0, 
0xC0, 0x7C, 0x7E, 0x38, 0x1C, 0x00, 0x60, 0x70, 
0x38, 0x1C, 0x0E, 0x07, 0x03, 0x81, 0xC0, 0xF8, 
0x60, 0x3E, 0x38, 0x1F, 0x8F, 0xC7, 0x03, 0x81, 
0xF8, 0xE0, 0x7C, 0x30, 0x1F, 0x1F, 0x8E, 0x07, 
0x00, 0x18, 0x0C, 0x0E, 0x07, 0x03, 0x81, 0xC0, 
0xE0, 0x70, 0x3F, 0x18, 0x0F, 0x86, 0x07, 0xE3, 
0xF1, 0xC0, 0xE0, 0x7E, 0x38, 0x1F, 0x8E, 0x07, 
0xC3, 0xE3, 0x81, 0xC0, 0x06, 0x03, 0x03, 0x81, 
0xC0, 0xE0, 0x70, 0x38, 0x1C, 0x0F, 0xC7, 0x03, 
0xE1, 0x81, 0xF8, 0xFC, 0x70, 0x38, 0x1F, 0x8E, 
0x07, 0xE3, 0x81, 0xF0, 0xF8, 0xE0, 0x70, 0x01, 
0x80, 0xC0, 0xE0, 0x70, 0x38, 0x1C, 0x0E, 0x07, 
0x03, 0xF1, 0xC0, 0xF8, 0x60, 0x7E, 0x3F, 0x1C, 
0x0E, 0x07, 0xE3, 0x81, 0xF8, 0xE0, 0x7C, 0x3E, 
0x38, 0x1C, 0x00, 0x70, 0x30, 0x18, 0x1C, 0x0E, 
0x07, 0x03, 0x81, 0xC0, 0xFC, 0x70, 0x3E, 0x18, 
0x0F, 0x8F, 0xC7, 0x03, 0x81, 0xF8, 0xE0, 0x7E, 
0x38, 0x1F, 0x0F, 0x86, 0x07, 0x00, 0x1C, 0x0C, 
0x06, 0x07, 0x03, 0x81, 0xC0, 0xE0, 0x70, 0x3F, 
0x1C, 0x0F, 0x86, 0x03, 0xE3, 0xF1, 0xC0, 0xE0, 
0x7E, 0x38, 0x1F, 0x8E, 0x07, 0xC3, 0xE1, 0x81, 
0xC0, 0x07, 0x03, 0x81, 0x80, 0xC0, 0xE0, 0x70, 
0x38, 0x1C, 0x0F, 0xC7, 0x03, 0xF1, 0xC0, 0xF8, 
0x7C, 0x70, 0x38, 0x1F, 0x8E, 0x07, 0xE3, 0x81, 
0xF8, 0xF8, 0x60, 0x30, 0x01, 0xC0, 0xE0, 0x60, 
0x30, 0x38, 0x1C, 0x0E, 0x07, 0x03, 0xF1, 0xC0, 
0xFC, 0x70, 0x3E, 0x1F, 0x1C, 0x0E, 0x07, 0xE3, 
0x81, 0xF8, 0xE0, 0x7E, 0x3E, 0x18, 0x0C, 0x00, 
0x70, 0x38, 0x18, 0x0C, 0x0E, 0x07, 0x03, 0x81, 
0xC0, 0xFC, 0x70, 0x3F, 0x1C, 0x0F, 0x87, 0xC7, 
0x03, 0x81, 0xF8, 0xE0, 0x7E, 0x38, 0x1F, 0x8F, 
0x86, 0x03, 0x00, 0x1C, 0x0E, 0x07, 0x03, 0x01, 
0x81, 0xC0, 0xE0, 0x70, 0x3F, 0x1C, 0x0F, 0xC7, 
0x03, 0xE1, 0xF0, 0xC0, 0xE0, 0x7E, 0x38, 0x1F, 
0x8E, 0x07, 0xE3, 0xF1, 0xC0, 0xC0, 0x07, 0x03, 
0x81, 0xC0, 0xC0, 0x60, 0x70, 0x38, 0x1C, 0x0F, 
0xC7, 0x03, 0xF1, 0xC0, 0xF8, 0x7C, 0x30, 0x38, 
0x1F, 0x8E, 0x07, 0xE3, 0x81, 0xF8, 0xFC, 0x70, 
0x30, 0x01, 0xC0, 0xE0, 0x70, 0x38, 0x18, 0x0C, 
0x0E, 0x07, 0x03, 0xF1, 0xC0, 0xFC, 0x70, 0x3F, 
0x1F, 0x0C, 0x06, 0x07, 0xE3, 0x81, 0xF8, 0xE0, 
0x7E, 0x3F, 0x1C, 0x0E, 0x00, 0x70, 0x38, 0x1C, 
0x0E, 0x06, 0x03, 0x01, 0x81, 0xC0, 0xFC, 0x70, 
0x3F, 0x1C, 0x0F, 0xC7, 0xC3, 0x01, 0x81, 0xF8, 
0xE0, 0x7E, 0x38, 0x1F, 0x8F, 0xC7, 0x03, 0x80, 
0x1C, 0x0E, 0x07, 0x03, 0x81, 0xC0, 0xC0, 0x60, 
0x70, 0x3F, 0x1C, 0x0F, 0xC7, 0x03, 0xF1, 0xF8, 
0xE0, 0x60, 0x3E, 0x38, 0x1F, 0x8E, 0x07, 0xE3, 
0xF1, 0xC0, 0xE0, 0x07, 0x03, 0x81, 0xC0, 0xE0, 
0x70, 0x30, 0x18, 0x1C, 0x0F, 0xC7, 0x03, 0xF1, 
0xC0, 0xFC, 0x7E, 0x38, 0x18, 0x0F, 0x8E, 0x07, 
0xE3, 0x81, 0xF8, 0xFC, 0x70, 0x38, 0x01, 0xC0, 
0xE0, 0x70, 0x38, 0x1C, 0x0C, 0x06, 0x03, 0x03, 
0xF1, 0xC0, 0xFC, 0x70, 0x3F, 0x1F, 0x8E, 0x06, 
0x03, 0xE1, 0x81, 0xF8, 0xE0, 0x7E, 0x3F, 0x1C, 
0x0E, 0x00, 0x70, 0x38, 0x1C, 0x0E, 0x07, 0x03, 
0x01, 0x80, 0xC0, 0xFC, 0x70, 0x3F, 0x1C, 0x0F, 
0xC7, 0xE3, 0x81, 0xC0, 0xF8, 0x60, 0x7E, 0x38, 
0x1F, 0x8F, 0xC7, 0x03, 0x80, 0x1C, 0x0E, 0x07, 
0x03, 0x81, 0xC0, 0xE0, 0x60, 0x30, 0x3F, 0x1C, 
0x01, 0x81, 0xC0, 0xE0, 0x70, 0x38, 0x1F, 0x8E, 
0x07, 0xC3, 0x01, 0xF0, 0xF8, 0xE0, 0x70, 0x3F, 
0x1C, 0x0F, 0xC7, 0x03, 0xE1, 0xF0, 0xC0, 0x60, 
0x03, 0x81, 0xC0, 0xC0, 0x60, 0x30, 0x38, 0x1C, 
0x0E, 0x07, 0xE3, 0x81, 0xF8, 0xE0, 0x7C, 0x3E, 
0x18, 0x1C, 0x0F, 0xC7, 0x03, 0xF1, 0xC0, 0xFC, 
0x7C, 0x30, 0x18, 0x00, 0xE0, 0x70, 0x30, 0x18, 
0x0C, 0x0E, 0x07, 0x03, 0x81, 0xF8, 0xE0, 0x7E, 
0x38, 0x1F, 0x0F, 0x86, 0x07, 0x03, 0xF1, 0xC0, 
0xFC, 0x70, 0x3F, 0x1F, 0x0C, 0x06, 0x00, 0x38, 
0x1C, 0x0C, 0x06, 0x03, 0x01, 0x81, 0xC0, 0xE0, 
0x7E, 0x38, 0x1F, 0x8E, 0x07, 0xC3, 0xE1, 0x81, 
0xC0, 0xFC, 0x70, 0x3F, 0x1C, 0x0F, 0xC7, 0xC3, 
0x01, 0x80, 0x0E, 0x07, 0x03, 0x81, 0x80, 0xC0, 
0x60, 0x70, 0x38, 0x1F, 0x8E, 0x07, 0xE3, 0x81, 
0xF0, 0xF8, 0x60, 0x30, 0x3F, 0x1C, 0x0F, 0xC7, 
0x03, 0xF1, 0xF8, 0xE0, 0x60, 0x03, 0x81, 0xC0, 
0xE0, 0x60, 0x30, 0x18, 0x1C, 0x0E, 0x07, 0xE3, 
0x81, 0xF8, 0xE0, 0x7C, 0x3E, 0x18, 0x0C, 0x0F, 
0xC7, 0x03, 0xF1, 0xC0, 0xFC, 0x7E, 0x38, 0x18, 
0x00, 0xE0, 0x70, 0x38, 0x18, 0x0C, 0x06, 0x03, 
0x03, 0x81, 0xF8, 0xE0, 0x7E, 0x38, 0x1F, 0x8F, 
0x86, 0x03, 0x01, 0xF1, 0xC0, 0xFC, 0x70, 0x3F, 
0x1F, 0x8E, 0x06, 0x00, 0x38, 0x1C, 0x0E, 0x07, 
0x03, 0x01, 0x80, 0xC0, 0xE0, 0x7E, 0x38, 0x1F, 
0x8E, 0x07, 0xE3, 0xE1, 0x80, 0xC0, 0x7C, 0x70, 
0x3F, 0x1C, 0x0F, 0xC7, 0xE3, 0x81, 0xC0, 0x0E, 
0x07, 0x03, 0x81, 0xC0, 0xC0, 0x60, 0x30, 0x18, 
0x1F, 0x8E, 0x07, 0xE3, 0x81, 0xF8, 0xFC, 0x60, 
0x30, 0x1F, 0x0C, 0x0F, 0xC7, 0x03, 0xF1, 0xF8, 
0xE0, 0x70, 0x03, 0x81, 0xC0, 0xE0, 0x70, 0x38, 
0x18, 0x0C, 0x06, 0x07, 0xE3, 0x81, 0xF8, 0xE0, 
0x7E, 0x3F, 0x1C, 0x0C, 0x07, 0xC3, 0x03, 0xF1, 
0xC0, 0xFC, 0x7E, 0x38, 0x1C, 0x00, 0xE0, 0x70, 
0x38, 0x1C, 0x0E, 0x06, 0x03, 0x01, 0x81, 0xF8, 
0xE0, 0x7E, 0x38, 0x1F, 0x8F, 0xC7, 0x03, 0x01, 
0xF0, 0xC0, 0xFC, 0x70, 0x3F, 0x1F, 0x8E, 0x07, 
0x00, 0x38, 0x1C, 0x0E, 0x07, 0x03, 0x81, 0x80, 
0xC0, 0x60, 0x3E, 0x38, 0x1F, 0x8E, 0x07, 0xE3, 
0xF1, 0xC0, 0xC0, 0x7C, 0x30, 0x1F, 0x1C, 0x0F, 
0xC7, 0xE3, 0x81, 0xC0, 0x0E, 0x07, 0x03, 0x81, 
0xC0, 0xE0, 0x70, 0x30, 0x18, 0x0F, 0x8E, 0x07, 
0xE3, 0x81, 0xF8, 0xFC, 0x70, 0x38, 0x1F, 0x0C, 
0x07, 0xC7, 0x03, 0xF1, 0xF8, 0xE0, 0x70, 0x03, 
0x81, 0xC0, 0xE0, 0x70, 0x38, 0x1C, 0x0C, 0x06, 
0x03, 0xE3, 0x81, 0xF8, 0xE0, 0x7E, 0x3F, 0x1C, 
0x0E, 0x07, 0xC3, 0x01, 0xF1, 0xC0, 0xFC, 0x7E, 
0x38, 0x1C, 0x00, 0xE0, 0x70, 0x38, 0x1C, 0x0E, 
0x07, 0x03, 0x01, 0x80, 0xF8, 0x60, 0x7E, 0x38, 
0x1F, 0x8F, 0xC7, 0x03, 0x81, 0xF0, 0xC0, 0x7C, 
0x30, 0x3F, 0x1F, 0x8E, 0x07, 0x00, 0x38, 0x1C, 
0x0E, 0x07, 0x03, 0x81, 0xC0, 0xE0, 0x60, 0x3E, 
0x18, 0x1F, 0x8E, 0x07, 0xE3, 0xF1, 0xC0, 0xE0, 
0x7C, 0x30, 0x1F, 0x0C, 0x0F, 0xC7, 0xE3, 0x81, 
0xC0, 0x0E, 0x07, 0x03, 0x81, 0xC0, 0xE0, 0x70, 
0x38, 0x18, 0x0F, 0x86, 0x07, 0xE3, 0x81, 0xF8, 
0xFC, 0x70, 0x38, 0x1F, 0x0C, 0x07, 0xC3, 0x03, 
0xF1, 0xF8, 0xE0, 0x70, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
]



# for i, byte in enumerate(data0):
#     print(f"{byte:08b}", end='')
#     if(i % 8 == 0):
#         print('')

raw_bits = ""
for byte in data0:
    raw_bits += f"{byte:08b}"

# print(raw_bits)

phase = 0
h_count = 0
l_count = 0
last_bit = 0
inverted = True
after_reset = False
parsed_bits = []

RS_LIMIT = 300
H0_LIMIT = 2
H1_LIMIT = 5
L0_LIMIT = 6
L1_LIMIT = 3
for bit in raw_bits:
    curr_bit = 1 if(bit == "0" and inverted) else 0

    if(last_bit == curr_bit):
        if(curr_bit == 1):
            h_count += 1
        else:
            l_count += 1
    else: # inversion happened # Reset Logic
        if(l_count >= RS_LIMIT or h_count >= RS_LIMIT):
            after_reset = True
        elif(last_bit == 0):
            if(l_count >= L0_LIMIT):
                parsed_bits.append(0)
            else:
                parsed_bits.append(1)
        else:
            if(h_count >= H1_LIMIT):
                parsed_bits.append(1)
            else:
                parsed_bits.append(0)
        h_count = 1 if(curr_bit == 1) else 0
        l_count = 1 if(curr_bit == 0) else 0

    last_bit = curr_bit

grb_tuples = []
TUP_BITS = 24
for i in range(0, len(parsed_bits), TUP_BITS):
    # Are we at the end? give up
    if(i >= (len(parsed_bits) - TUP_BITS)):
        break
    green = int("".join(str(e) for e in parsed_bits[i:i+8]),2)
    red   = int("".join(str(e) for e in parsed_bits[i+8:i+16]),2)
    blue  = int("".join(str(e) for e in parsed_bits[i+16:i+24]),2)
    grb_tuples.append((green, red, blue))

print(grb_tuples)









# Sample bits
# taken @ approx 7.8 MHz
# roughly 1.28E-7 seconds per bit
# 0.128 usec resolution
# reset is after >=50 usec
# reset takes 391 bits (390.625 rounded up)
# 1110000001110000001111110001110000001111110001111100001100000001
# 1000000000001110000001110000001100000001100000001100000001100000
# 0111000000111000000111000000111000000111111000111000000111110000
# 1111100001100000011100000011100000011100000011111100011100000011
# 1111000111110000110000000110000000000011100000011100000011100000
# 0110000000110000000110000001110000001110000001110000001110000001
# 1111100011100000011111000011111000011000000111000000111000000111
# 0000001111110001110000001111110001111110001110000001100000000000
# 1110000001110000001110000001100000001100000011100000011100000011
# 1000000111000000111000000111111000111000000111111000111110000110
# 0000011100000011100000011100000011111100011100000011111100011111
# 1000111000000111100000000011100000011100000011100000011100000011

# 111 - approx .38 us, a 0H
# 0000 - approx .51 us,  1L
# 0000000 - approx .89 us, a 0L
# 111111 -approx .77 us, a 1H
# 3-4 chars are likely the short, 6+ are likely the long



# Optimal for write is likely:
# NOTE: inverted from read (using the voltage amplifier)
# 0H: 000      ~0.38 usec (0.35 usec) within tolerance (+/- 0.15 usec)
# 1H: 000000   ~0.77 usec (0.70 usec) within tolerance (+/- 0.15 usec)
# 0L: 1111111  ~0.90 usec (0.80 usec) within tolerance (+/- 0.15 usec)
# 1L: 11111    ~0.64 usec (0.60 usec) within tolerance (+/- 0.15 usec)
