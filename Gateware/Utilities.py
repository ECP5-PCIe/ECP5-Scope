from amaranth import *
from amaranth.build import *

__all__ = ["LFSRAddressGenerator", "DeltaSigmaDAC"]

class LFSRAddressGenerator(Elaboratable):
    """Linear-feedback shift register address generator

    Parameters
    ----------
    n : int
        Length of the LFSR

    Attributes
    ----------
    n : int
        Length of the LFSR
    state : Signal(n)
        LFSR state
    reset : Signal()
        Reset, independent of enable
    enable : Signal()
        Enable the LFSR
    """
    def __init__(self, n):
        assert n >= 2
        self.n = n
        self.state = Signal(n, reset = 1)
        self.reset = Signal()
        self.enable = Signal()

    def elaborate(self, platform):
        m = Module()

        n = self.n

        # See https://en.wikipedia.org/wiki/Linear-feedback_shift_register
        polynomials = [0, 0, 0x3, 0x6, 0xC, 0x14, 0x30, 0x60, 0xB8, 0x110, 0x240, 0x500, 0xE08, 0x1C80, 0x3802, 0x6000, 0xD008, 0x12000, 0x20400, 0x72000, 0x90000, 0x140000, 0x300000, 0x420000, 0xE10000]

        # I think I found this by trial and error? It works
        polynomial = (polynomials[n] << 1) & (2 ** n - 1)

        bit = self.state[0]
        # If the bit is 1, then XOR it with the polynomial, this is equivalent to XORing every position in the polynomial with the bit
        new_state = self.state ^ Mux(bit, polynomial, 0)
        
        m.d.sync += self.state.eq(Mux(self.reset, self.state.reset, Mux(self.enable, new_state.rotate_right(1), self.state)))

        return m



class DeltaSigmaDAC(Elaboratable):
    """Delta Sigma DAC
    Parameters
    ----------
    value : Signal(nbits)
        Signal to analogize
    output : Signal()
        Output signal
    order : int
        Order of the converter (>= 3 doesn't work)
    nbits : int
        Number of bits of the DAC
    """
    def __init__(self, value, output, order=1, nbits=24):
        assert(len(value) == nbits) # nbits is redundant, maybe remove
        self.value = value
        self.output = output
        self.nbits = nbits
        self.order = order
        self.counters = [Signal(nbits + 2 + i) for i in range(order)]

    def elaborate(self, platform: Platform) -> Module:
        m = Module()

        nbits = self.nbits
        counters = [self.value, *self.counters]
        # Output is true when the counter overflows
        # For example if the maximum value is 255 and you add 250 every cycle, then it overflows 98 % of cycles and outputs 0.98 Vcc
        m.d.sync += self.output.eq(counters[-1][-1])

        for order in range(1, self.order + 1):
            counter = counters[order]
            # Increase the counter by the input value each cycle, if it overflows, then subtract one overflow bit
            m.d.sync += counter.eq(counter + counters[order - 1] - Mux(counters[-1][-1], 1 << (self.nbits + order - 1), 0))

        return m