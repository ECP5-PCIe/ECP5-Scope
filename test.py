# Math playground

state = 0b1 << 7 | 1

from collections import Counter

res = []
#print(state)

#for polynomial in range(256):
n = 8

# See https://en.wikipedia.org/wiki/Linear-feedback_shift_register
polynomials = [0, 0, 0x3, 0x6, 0xC, 0x14, 0x30, 0x60, 0xB8, 0x110, 0x240, 0x500, 0xE08, 0x1C80, 0x3802, 0x6000, 0xD008, 0x12000, 0x20400, 0x72000, 0x90000, 0x140000, 0x300000, 0x420000, 0xE10000]

polynomial = (polynomials[n] << 1) & 0xFF
res = []
state = 1
res.append(state)

for i in range(2 ** n):
    bit = state & 1
    new_state = (state ^ (polynomial if bit else 0))
    state = new_state >> 1
    state |= (new_state & 1) << (n - 1)

    #if state == 1:
    #    print(i, state)
    print(i, hex(state))
    res.append(state)

#if(len(Counter(res)) == 255):
print(bin(polynomial), len(Counter(res)))
print(bin((0x1 << n) + polynomial)[3:], len(Counter(res)))