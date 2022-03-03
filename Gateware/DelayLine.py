from amaranth import *
from amaranth.build import *
from Utilities import LFSRAddressGenerator
import math


class DelayLine(Elaboratable):
    # Based on https://gist.github.com/newhouseb/784cc0c24f8681c3224c15758be5d1b8
    # Is it actually sampled in an instant? The clock tree might have some propagation delay
    # Essentially a 55 GS/s logic analyzer
    """
    LUT-based delay line
    The input signal propagates across the delay line and is sampled once each clock cycle in an instant.

    Parameters
    ----------
    length: int
        Length of the delay line, must be divisible by two

    Attributes
    ----------
    input_signal: Signal()
        Signal at the input
    length: int
        Length of the delay line, must be divisible by two
    output_signal: Signal(length)
        Value of the delay line
    """
    def __init__(self, length):
        assert length % 2 == 0 # Not absolutely necessary, though otherwise the last bit will always be 0
        self.input_signal = Signal()
        self.length = length
        self.output_signal = Signal(length)

    def elaborate(self, platform):
        m = Module()

        # Iterator for the slice positions
        def get_slice():
            start_x = 10
            start_y = 11
            slice_i = 0

            while True:
                yield "X{}/Y{}/SLICE{}".format(start_x, start_y, ["A","B","C","D"][slice_i])

                if slice_i == 3:
                    start_x += 1

                slice_i = (slice_i + 1) % 4


        gen_slice = get_slice()

        intoff = Signal(self.length + 1)
        carry = Signal(self.length // 2 + 1)

        m.submodules.delay_start = Instance("TRELLIS_SLICE",
            a_BEL = next(gen_slice),

            p_MODE           = "CCU2",
            p_CCU2_INJECT1_0 = "NO",
            p_CCU2_INJECT1_1 = "YES",
            p_LUT0_INITVAL   = 0x00FF,
            p_LUT1_INITVAL   = 0x00FF,
            i_D1             = self.input_signal,
            o_FCO            = carry[0]
        )

        for i in range(self.length // 2):
            delay = Instance("TRELLIS_SLICE",
                a_BEL = next(gen_slice),

                p_MODE           = "CCU2",
                p_CCU2_INJECT1_0 = "NO",
                p_CCU2_INJECT1_1 = "NO",
                p_LUT0_INITVAL   = 0x3300,
                p_LUT1_INITVAL   = 0x3300,
                p_REG0_SD        = "1",
                p_REG1_SD        = "1",

                i_B0  = 0,
                i_B1  = 0,
                i_LSR = 0,

                o_F0  = intoff[2 * i + 0], # Out of the LUT
                i_DI0 = intoff[2 * i + 0], # Into the FF
                o_Q0  = self.output_signal[2 * i + 0], # Out of the FF

                o_F1  = intoff[2 * i + 1],
                i_DI1 = intoff[2 * i + 1], # Not i_F0?
                o_Q1  = self.output_signal[2 * i + 1],

                i_FCI = carry[i],
                o_FCO = carry[i + 1],

                i_CLK = ClockSignal()
            )
            setattr(m.submodules, "delay_" + str(i), delay)

        return m



class Recorder(Elaboratable):
    """
    Parameters:
    width: int
        Length of the delay line
    depth: int
        Memory depth, Sample depth = width * depth, has to be 2 ^ n - 1
    input_signal: Signal()
        Input signal to sample
    read_port:
        Memory read port, read data from here, generate addresses with LFSR
    start: Signal()
        Start sampling data
    sampling: Signal()
        Whether it is currently sampling data
    """
    def __init__(self, width, depth):
        assert math.log2(depth + 1) == int(math.log2(depth + 1)) # Restricted by the LFSR, since it goes over all states besides 0 in n bits

        self.memory = Memory(width = width, depth = depth + 1)
        self.delay_line = DelayLine(width)
        self.input_signal = self.delay_line.input_signal
        self.read_port = self.memory.read_port(transparent = False) # Read data from this
        self.start = Signal()
        self.sampling = Signal()

    def elaborate(self, platform):
        m = Module()

        m.submodules += self.delay_line
        #m.submodules.read_port = self.memory.read_port(transparent = False)
        m.submodules.write_port = write_port = self.memory.write_port()
        m.submodules.lfsr = lfsr = LFSRAddressGenerator(len(write_port.addr))

        #m.d.comb += lfsr.reset.eq(~(self.start | self.sampling))
        m.d.comb += lfsr.reset.eq(~self.sampling)
        m.d.comb += lfsr.enable.eq(1)
        
        with m.If(self.start):
            m.d.comb += lfsr.reset.eq(0)
            m.d.sync += self.sampling.eq(1)
        
        with m.If(self.sampling & (lfsr.state == 1)):
            m.d.sync += self.sampling.eq(0)

        sampling_signal = Signal(len(self.delay_line.output_signal))

        # Maybe this could be used as another layer of FFs inbetween
        m.d.sync += sampling_signal.eq(self.delay_line.output_signal)

        # Use a linear-feedback shift register instead of a counter for the addresses, since it is faster.
        # This can run at over 500 MHz on an LFE5UM5G-85F, if the DACs are in a slower clock domain.
        m.d.sync += write_port.addr.eq(lfsr.state)
        m.d.sync += write_port.data.eq(self.delay_line.output_signal) # TODO: Is sampling_signal better?
        #m.d.sync += write_port.data.eq(sampling_signal)
        m.d.sync += write_port.en.eq(self.sampling)

        return m



class TDC(Elaboratable):
    # Time to digital converter, for example for triggering, with some example code.
    # This is probably necessary to build a sampling oscilloscope without a phase coherent reference between the sampling clock and the signal

    #Use Decoder to one hot decode the delay line


    def elaborate(self, platform):
        m = Module()

        #Hmm, multistage binary search? This algorithm should find the last transition in the delay line

        delay_in = Signal(256)

        search_1_has_transition = Signal(128)
        search_1_which = Signal(128)

        m.d.sync += search_1_has_transition.eq(delay_in[0 : len(delay_in) : 2] ^ delay_in[1 : len(delay_in) : 2]) # Bit is high if the last stage had a transition there
        m.d.sync += search_1_which.eq(delay_in[1 : len(delay_in) : 2]) # Which bit went high


        search_2_has_transition = Signal(64)
        search_2_which_1 = Signal(64)
        search_2_which_2 = Signal(64)
        arr_len = Len(search_1_has_transition)

        m.d.sync += search_2_has_transition.eq(search_1_has_transition[0 : arr_len : 2] | search_1_has_transition[1 : arr_len : 2])
        m.d.sync += search_2_which_1.eq(
            (search_1_which[0 : arr_len : 2] & search_1_has_transition[0 : arr_len : 2]) |
            (search_1_which[1 : arr_len : 2] & search_1_has_transition[1 : arr_len : 2]))
        m.d.sync += search_2_which_2.eq(search_1_has_transition[1 : arr_len : 2]) # Which subdivision of the first search has the transition

        #etc...



        #Alternatively: Find position while acquiring data one by one


        delay_in = Signal(xyz)

        filtered_in = delay_in[1:] | delay_in[:-1] # Remove single glitches

        position = Signal(range(len(delay_in)))

        counter = Signal(len(position), reset = 1)

        m.d.sync += counter.eq(counter + 1)

        with m.If(filtered_in.bit_select(counter, 1) & ~filtered_in.bit_select(counter - 1, 1)):
            m.d.sync += position.eq(counter)
            m.d.sync += done.eq(1)

        return m