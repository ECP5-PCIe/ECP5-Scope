from amaranth import *
from amaranth.build import *
from amaranth.lib.cdc import FFSynchronizer
from amaranth_boards import ecp5_5g_evn as FPGA
from amaranth_stdio.serial import AsyncSerial
from DelayLine import Recorder
from Utilities import *

##### WARNING: Read the warnings, this might damage or destroy your ECP5 if connected wrong

class Oscilloscope(Elaboratable):
    def __init__(self):
        pass

    def elaborate(self, platform):
        m = Module()

        # IO section
        global pin_i
        pin_i = 0
        def get_pin(pin, attrs):
            global pin_i
            pin_i += 1
            platform.add_resources([Resource("pins", pin_i, pin, attrs)])
            return platform.request("pins", pin_i)
        
        ###### WARNING: DO NOT CONNECT MORE THAN 1.2 V TO VCCIO7, IT COULD DESTROY YOUR CHIP (1.5 V should be okay, but might risk electromigration if the code is modified, 1.8 V miiiight work but should use the 18 instead of the 15 IO_TYPE)
        # Why? A thevenin termination is 100 Ω to each rail, which is parallel 50 Ω at the input and series 200 Ω at the rails. This dissipates power:
        # 7.2 mW @ 1.2 V
        # 11.25 mW @ 1.5 V
        # 16.2 mW @ 1.8 V
        # Remember: The resistors are on the chip, their size in the order of micrometers,
        # a 01005 resistor can usually only dissipate 31 mW and the on-chip resistors are likely much smaller.
        # On the other hand, silicon has a rather high thermal conductivity compared to PCB substrate and air, so it kind of evens out a little.
        #
        # Feel free to play around with the IO types to see if you can get better waveforms
        test_out     = get_pin(Pins("G20", dir="o")          , Attrs()).o
        test_pin     = get_pin(Pins("F4", dir="io")           , Attrs(IO_TYPE="SSTL15_II", SLEWRATE="FAST", TERMINATION="50"))
        #test_pin_2   = get_pin(Pins("E3", dir="o")          , Attrs(IO_TYPE="SSTL15_II", SLEWRATE="FAST", TERMINATION="150"))
        trig_clk_in  = get_pin(DiffPairs("A3", "B3", dir="i"), Attrs(IO_TYPE="LVDS")).i
        signal       = get_pin(DiffPairs("C3", "D3", dir="i"), Attrs(IO_TYPE="SSTL15D_II", TERMINATION="50"))
        dac_trig     = get_pin(Pins("D5", dir="io")          , Attrs(IO_TYPE="LVCMOS15", SLEWRATE="FAST")) # 15 µA leakage, this was dac_sig before, my board might have a fault
        dac_sig      = get_pin(Pins("E4", dir="io")          , Attrs(IO_TYPE="LVCMOS15", SLEWRATE="FAST"))

        signal_in = signal.i



        ### Clock section

        # 300 MHz in this case
        m.domains += ClockDomain("sample")

        fb = Signal()
        clk100 = Signal()
        clk_hf = Signal()

        freq_sync = 12E+6 # 12 MHz at ClockSignal("sync")
        vco_mul = 100 # 1200 MHz at VCO
        sample_div = 4 # 300 MHz at ClockSignal("sample")

        freq_sample = freq_sync * vco_mul / sample_div

        m.submodules += Instance("EHXPLLL",
            p_OUTDIVIDER_MUXA='DIVA',
            p_CLKOP_ENABLE='ENABLED',
            p_CLKOS_ENABLE='ENABLED',
            p_CLKOS2_ENABLE='ENABLED',
            p_CLKOS3_ENABLE='ENABLED',

            p_CLKOP_DIV=vco_mul, # 1200 MHz VCO
            p_CLKFB_DIV=1,
            p_CLKI_DIV=1,

            p_CLKOS_DIV=sample_div, # 300 MHz sample clock
            p_CLKOS2_DIV=12, # 100 MHz
            p_CLKOS3_DIV=1, # 1200 MHz

            p_FEEDBK_PATH='CLKOP', # Does INT_OP yield a different result? Kinda, swap p_CLKOP_DIV and p_CLKFB_DIV to try
            i_CLKI=ClockSignal("sync"),
            i_ENCLKOP=1,
            i_ENCLKOS=1,
            i_ENCLKOS2=1,
            i_ENCLKOS3=1,
            o_CLKOS=ClockSignal("sample"),
            o_CLKOS2=clk100,
            o_CLKOS3=clk_hf,

            # Stuff, this might improve phase noise. These are random values, not based on any empirical measurement. Does somebody have a phase noise analyzer?
            a_ICP_CURRENT="12",
            a_LPF_RESISTOR="8",
            a_MFG_ENABLE_FILTEROPAMP="1",
            a_MFG_GMCREF_SEL="2",


            i_CLKFB=fb,
            o_CLKOP=fb,
            )


        # Test signals PLL

        fb2 = Signal()
        fb3 = Signal()
        test_clk = Signal()
        #m.d.comb += test_clk.eq(ClockSignal("sync") ^ ~ClockSignal("sync"))

        if False:
            m.submodules += Instance("EHXPLLL",
                a_LOC = "PLL0", # Location, one of PLL0, 1, 2, 3
                p_OUTDIVIDER_MUXA='DIVA',
                p_CLKOP_ENABLE='ENABLED',
                p_CLKOS_ENABLE='ENABLED',
                p_CLKOS2_ENABLE='ENABLED',

                p_CLKOP_DIV=1,
                p_CLKFB_DIV=10,
                p_CLKI_DIV=1,

                p_CLKOS_DIV=1,
                p_CLKOS2_DIV=20,

                p_FEEDBK_PATH='CLKOP',
                i_CLKI=fb3,
                i_ENCLKOP=1,
                i_ENCLKOS=1,
                i_ENCLKOS2=1,
                o_CLKOS=test_clk,
                o_CLKOS2=fb3,

                #a_ICP_CURRENT="12",
                #a_LPF_RESISTOR="127",
                #a_MFG_ENABLE_FILTEROPAMP="1",
                #a_MFG_GMCREF_SEL="2",


                i_CLKFB=fb2,
                o_CLKOP=fb2,
                )
        
        else:
            m.submodules += Instance("EHXPLLL",
                a_LOC = "PLL0", # Location, one of PLL0, 1, 2, 3
                p_OUTDIVIDER_MUXA='DIVA',
                p_CLKOP_ENABLE='ENABLED',
                p_CLKOS_ENABLE='ENABLED',

                p_CLKOP_DIV=10,
                p_CLKFB_DIV=1,
                p_CLKI_DIV=1,

                p_CLKOS_DIV=5,

                p_FEEDBK_PATH='CLKOP',
                i_CLKI=clk100,
                i_ENCLKOP=1,
                i_ENCLKOS=1,
                i_ENCLKOS2=1,
                o_CLKOS=test_clk,

                #a_ICP_CURRENT="12",
                #a_LPF_RESISTOR="127",
                #a_MFG_ENABLE_FILTEROPAMP="1",
                #a_MFG_GMCREF_SEL="2",


                i_CLKFB=fb2,
                o_CLKOP=fb2,
                )
        
        #m.d.comb += test_signal.eq(clk100)
        
        #m.submodules += Instance("DCSC",
        #    i_CLK0 = ClockSignal(),
        #    i_CLK1 = ~test_clk,
        #    o_DCSOUT = test_clk,
        #    i_SEL0 = 0b0,
        #    i_SEL1 = 0b1,
        #    i_MODESEL = 0b1,
        #    )
        
        #m.submodules.counter = counter = AsynchronousGrayCodeCounter()

        #platform.add_resources([Resource("test", 1, DiffPairs("A4", "A5", dir="o"))])
        #m.d.comb += platform.request("test", 1).o.eq(counter.async_gray_count[-1])
        #m.d.comb += test_clk.eq(counter.async_gray_count[-1])



        ### Oscilloscope section

        delay = 17E-12 # 1 ps extra, delay per LUT, 18 ps on my device
        clock_frequency = freq_sample
        width = (int(1 / (clock_frequency * delay)) + 1) // 2 * 2
        depth = 255 # Memory depth, 255 samples

        # Device info
        print(f"Memory format: {width}x{depth}, {width * depth} bits at {clock_frequency / 1E6} MHz")
        print(f"Sample time: {width * depth * 18E-6} µs")
        print(f"Length for TDR at 0.7 c: {3e8 * width * depth * 18E-12 * 0.7 * 0.5:.1f} m")
        print(f"Time per frame at 2 MBaud and 256 voltages: {(width + 7) // 8 * depth * 10 / 2e6 * 256} s") # This is too fast, in reality almost half as fast
        print()

        # This is where the magic happens, head over to DelayLine.py to find out more
        recorder = m.submodules.recorder = DomainRenamer("sample")(Recorder(width, depth))
        
        # Maybe there could be a mux here, to calibrate the delay line.
        # Maybe there could be another gateware, which measures the delay per LUT and then auto-generates the delay and clock frequency parameters.
        #m.d.comb += recorder.input_signal.eq(test_clk)
        m.d.comb += recorder.input_signal.eq(signal_in)

        # Signals in sync domain
        start = Signal()
        sampling = Signal()

        # Synchronize the signals
        m.submodules += FFSynchronizer(start, recorder.start, o_domain = "sample")
        m.submodules += FFSynchronizer(recorder.sampling, sampling, o_domain = "sync")

        
        # This outputs a warning, this is the read port of the memory of the Recorder
        read_port = m.submodules.read_port = recorder.read_port

        # Data is not written sequentially into the memory, but rather in locations given by a LFSR.
        # This limits the memory size in steps of 2 ** n - 1 (2 ** n could be doable with some tricks).
        m.submodules.lfsr = lfsr = LFSRAddressGenerator(len(read_port.addr))
        m.d.comb += read_port.addr.eq(lfsr.state)
        m.d.comb += read_port.en.eq(1)

        # Delta Sigma DAC converters
        ref_sig = Signal(24)
        m.submodules += DomainRenamer("sample")(DeltaSigmaDAC(ref_sig, dac_sig.o, order=1, nbits=24))

        ref_trig = Signal(24)
        m.submodules += DomainRenamer("sample")(DeltaSigmaDAC(ref_trig, dac_trig.o, order=1, nbits=24))

        # For disabling the DAC during acquisition, since they are pretty noisy. This might cause their values to drift due to leakage.
        enable_dac = Signal()
        m.d.comb += [dac_sig.oe.eq(enable_dac), dac_trig.oe.eq(enable_dac)] # Maybe disable converters too since otherwise they might drift apart a little

        # When start or sampling are true, it is acquiring data.
        # In the final plot, a noisy area can be seen at the beginning before it turns off, about 100 ns into the acquisition. (It should be much sooner? Why does it take so long?)
        m.d.sync += enable_dac.eq(~(start | sampling))


        # Enable the step or pulse signal generator output
        # This is there to avoid electromigration, since a DC current over 10 mA should be avoided, so the en_pulse signal is only on for about 1 ms.
        # Since the signal is terminated with 50 Ω to 0.6 V, connecting it to ground will pass about 12 mA
        en_pulse = Signal()

        #en_pulse_s = Signal()
        #m.submodules += FFSynchronizer(en_pulse, en_pulse_s, o_domain = "sample")
        #lrs = Signal(1)
        #m.d.sample += lrs.eq(Cat(recorder.sampling, lrs[:-1]))
        ##m.d.sample += test_pin.o.eq(~lrs[-1] & recorder.sampling)

        #m.d.comb += test_pin.oe.eq(1)
        if True:
            if False: # Pulse
                m.domains += ClockDomain("hf")
                m.d.comb += ClockSignal("hf").eq(clk_hf)
                sampling_hf = Signal()
                m.submodules += FFSynchronizer(recorder.sampling, sampling_hf, o_domain = "hf")
                last_sampling_hf = Signal()
                m.d.hf += last_sampling_hf.eq(sampling_hf)
                m.d.hf += test_pin.o.eq(~last_sampling_hf & sampling_hf)

            else: # Step
                # Voltages with the test signal connected via 330 Ω to the input signal
                # 0 = 0.37 Vcc
                # x = 0.45 Vcc
                # 1 = 0.56 Vcc

                enable_output = Signal()

                last_start = Signal()
                m.d.sample += last_start.eq(recorder.start)

                cnt = Signal(16)

                with m.If(last_start & ~recorder.start):
                    m.d.sample += enable_output.eq(0)

                last_sampling = Signal()
                m.d.sample += last_sampling.eq(recorder.sampling)

                with m.If(last_sampling & ~recorder.sampling):
                    m.d.sample += enable_output.eq(1)

                # If you have a BFU730F or so, instead of connecting both pins with 330 Ω, then set this to True.
                # If you don't have it but this is True, then there will be more spurious oscillations in the signal.
                # That is because then the test signal is terminated at 0 Ω (OE = 1) instead of 50 Ω (OE = 0)
                transistor = True
                if transistor:
                    m.d.sample += test_pin.oe.eq(en_pulse)
                    m.d.sample += test_pin.o.eq(enable_output)
                else:
                    m.d.sample += test_pin.oe.eq(enable_output & en_pulse)
                    m.d.sample += test_pin.o.eq(0)
                #m.d.comb += test_pin.o.eq(0)
        
        else:
            m.d.comb += test_pin.oe.eq(ClockSignal())
            m.d.comb += test_pin.o.eq(0)
            m.d.comb += test_out.eq(clk100)



        # PC interface section

        uart = m.submodules.uart = AsyncSerial(divisor = int(6), pins = platform.request("uart", 0)) # 2 MBaud @ 12 MHz

        m.d.comb += uart.rx.ack.eq(1)

        # Commands:
        # S: Sample and send data
        #    Returns 4 bytes `width`, 4 bytes `depth` (LS bit and byte first)
        #    Returns data stream, where each `(width + 7) // 8` bytes is one sample block, only the first `width` bytes matter
        #    It has `depth` blocks in total
        #
        # R x [w]: Register command
        #          x is the address, bit #7 = 1 for write
        #          w should be present if x[7] == 1, four bytes data to write to that address
        #          If x[7] == 0, then four bytes will be returned which is the data stored at that address

        # Number of bytes to send per row
        n_bytes = (width + 7) // 8

        enwider = Signal(n_bytes * 8) # Enlarge to 8 bit units for word_select, might be unnecessary
        m.d.comb += enwider.eq(read_port.data)

        word_select = Signal(range(n_bytes))
        word_select_max = n_bytes - 1

        word = enwider.word_select(word_select, 8)

        width_sig = Signal(32, reset = width)
        depth_sig = Signal(32, reset = depth)
        param_word_counter = Signal(2)

        # Configuration registers, 32 bit
        registers = Cat([
            ref_sig , Signal(8), # 0, signal reference voltage
            ref_trig, Signal(8), # 1, trigger reference voltage
        ])

        reg_addr = Signal(8) # Bit 7 true = write
        reg_cnt = Signal(2)

        timer = Signal(32)
        m.d.sync += timer.eq(timer + 1)

        with m.FSM(name="Debug"):
            with m.State("Start"):
                with m.If(uart.rx.rdy):
                    with m.If(uart.rx.data == ord("S")): # Get samples
                        m.next = "Sample-Start"

                    with m.If(uart.rx.data == ord("P")): # Get samples with pulse
                        m.d.sync += timer.eq(0)
                        m.next = "Pulse-Start"

                    with m.If(uart.rx.data == ord("R")): # Read register
                        m.next = "Register-Address"


            with m.State("Register-Address"):
                with m.If(uart.rx.rdy):
                    m.d.sync += reg_cnt.eq(0) # Redundant
                    m.d.sync += reg_addr.eq(uart.rx.data)
                    m.next = "Register-Data"

            with m.State("Register-Data"):
                with m.If(reg_addr[7]):
                    with m.If(uart.rx.rdy):
                        m.d.sync += reg_cnt.eq(reg_cnt + 1)
                        # Iterates over each byte in the register address
                        m.d.sync += registers.word_select(Cat(reg_cnt, reg_addr[:7]), 8).eq(uart.rx.data)

                        with m.If(reg_cnt == 3):
                            m.d.sync += reg_cnt.eq(0) # Redundant
                            m.next = "Start"
                
                with m.Else():
                    with m.If(uart.tx.rdy):
                        m.d.sync += reg_cnt.eq(reg_cnt + 1)
                        m.d.comb += uart.tx.data.eq(registers.word_select(Cat(reg_cnt, reg_addr[:7]), 8))
                        m.d.comb += uart.tx.ack.eq(1)

                        with m.If(reg_cnt == 3):
                            m.d.sync += reg_cnt.eq(0) # Redundant
                            m.next = "Start"


            with m.State("Pulse-Start"):
                m.d.sync += en_pulse.eq(1)
                with m.If(timer == 12000): # 1 ms, wait for the connected device to settle
                    m.next = "Sample-Start"

                
            with m.State("Sample-Start"):
                m.d.sync += start.eq(1)

                with m.If(sampling):
                    m.d.comb += lfsr.enable.eq(1) # Shift reading LFSR by one
                    m.d.sync += start.eq(0)
                    m.d.sync += param_word_counter.eq(0)
                    m.next = "Sample-Wait-1"

                with m.Else():
                    m.d.comb += lfsr.reset.eq(1) # Reset the LFSR

                
            with m.State("Sample-Wait-1"): # Transmit parameter: width
                with m.If(uart.tx.rdy):
                    m.d.comb += uart.tx.data.eq(width_sig.word_select(param_word_counter, 8))
                    m.d.comb += uart.tx.ack.eq(1)
                    m.d.sync += param_word_counter.eq(param_word_counter + 1)

                    with m.If(param_word_counter == 3):
                        m.d.sync += param_word_counter.eq(0) # Redundant
                        m.next = "Sample-Wait-2"
                
            with m.State("Sample-Wait-2"): # Transmit parameter: depth
                with m.If(uart.tx.rdy):
                    m.d.comb += uart.tx.data.eq(depth_sig.word_select(param_word_counter, 8))
                    m.d.comb += uart.tx.ack.eq(1)
                    m.d.sync += param_word_counter.eq(param_word_counter + 1)

                    with m.If(param_word_counter == 3):
                        m.d.sync += param_word_counter.eq(0) # Redundant
                        m.next = "Sample-Wait-3"

            with m.State("Sample-Wait-3"): # Wait for the delay line to finish, should have already happened unless the UART is extremely fast.
                with m.If(~sampling):
                    m.d.sync += en_pulse.eq(0)
                    m.next = "Send"
            
            with m.State("Send"): # Send the memory over UART
                with m.If(uart.tx.rdy):
                    m.d.comb += uart.tx.data.eq(word)
                    m.d.comb += uart.tx.ack.eq(1)

                    with m.If(word_select == word_select_max):
                        m.d.sync += word_select.eq(0)
                        m.d.comb += lfsr.enable.eq(1)

                        with m.If(lfsr.state == 1): # Jump to Start when it is at the last address
                            m.next = "Start"

                    with m.Else():
                        m.d.sync += word_select.eq(word_select + 1)


        return m



if __name__ == "__main__":
    # Make sure that the user knows to set VCCIO7 to the proper voltage
    print("Is VCCIO7 at 1.2 V?")
    assert input("y/n ").lower() == "y"
    print("Are you sure? This might destroy your ECP5 if it is not the case")
    assert input("y/n ").lower() == "y"
    print("Did you hook it up to another voltage than 1.2 V? Since then the on-chip termination resistors may overheat and melt")
    assert input("y/n ").lower() == "n"

    FPGA.ECP55GEVNPlatform().build(Oscilloscope(), do_program=True, nextpnr_opts="-r")

    with open("build/top.tim") as logfile:
        for line in logfile.read().split("\n"):
            if "Max frequency for clock" in line:
                print()
                print(line)
                print("Clock frequency:", line.split("MHz")[1].split()[-1], "MHz, max:", line.split("MHz")[0].split()[-1], "MHz")