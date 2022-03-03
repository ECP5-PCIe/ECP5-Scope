import serial
from glob import glob
import numpy as np
import matplotlib.pyplot as plt
import time
import scipy.signal
from matplotlib.colors import LinearSegmentedColormap

port = serial.Serial(port=glob("/dev/serial/by-id/usb-Lattice_Lattice_ECP5_Evaluation_Board_*-if01-port0")[0], baudrate=2000000)

def read_buffer():
    # Send pulse and acquire command
    port.write(b"P")

    # First two values in the data stream are the dimensions of the line
    width = int.from_bytes(port.read(4), byteorder="little")
    depth = int.from_bytes(port.read(4), byteorder="little")

    # Number of bytes in the data stream per line
    n_bytes = (width + 7) // 8
    # Debug
    if False:
        print("w", width)
        print("d", depth)
        print("n", n_bytes)

    result = np.zeros((depth, width), dtype=bool) # Could be a list
    for i in range(depth):
        data = port.read(n_bytes)

        for j in range(width):
            # One location in result is one bit in data, so go through it bit by bit
            result[i, width - j - 1] = ((data[j // 8] >> j % 8) & 1) == 1
            
    return result


def read_register(address):
    assert 0 <= address <= 127
    # R is the register command, bit 7 = 0 to read
    port.write(b"R" + address.to_bytes(1, "little"))
    return int.from_bytes(port.read(4), byteorder="little")


def write_register(address, data):
    assert 0 <= address <= 127
    # R is the register command, bit 7 = 1 to write
    port.write(b"R" + (0x80 + address).to_bytes(1, "little") + data.to_bytes(4, "little"))


def acquire():
    # Minimum and maximum voltages in terms of Vcc
    a = 0.0
    b = 0.7
    # n is the number of voltage steps
    # d is the number of averages, so d = 2 means that the same voltage is sampled twice and then averaged
    n = 128
    d = 2
    # Time and voltage blue
    t_blur_n = 30
    u_blur_n = 5

    # Reset reference voltage
    write_register(0, int(a * 2 ** 24))
    # Wait for RC to settle
    time.sleep(1)

    # Start time for performance measurement
    start = time.clock_gettime_ns(0)
    acquisitions = []
    # These will get removed later, could be replaced by zeros
    for i in range(u_blur_n):
        data = read_buffer()

        decim = data.shape[1]
        decim = 186

        reduced = data[:, 5:5+decim].flatten()
        acquisitions.append(reduced)

    # Iterate through the voltages
    # This could be replaced by a better algorithm, which does a binary-search-like scan, maybe based on a histogram, which has more samples in areas where there are more changes, or none where nothing changes
    for i in range(n):
        print(f"\n{i + 1} of {n} {(i + 1) / n * 100:.1f}%        ", end="\r\u008D")
        # i / (n - 1) goes from 0% to 100%, * (b - a) + a rescales it from [0, 1] to [b, a], * (2 ** 24 - 1) turns it into the ADC value
        write_register(0, int((i / (n - 1) * (b - a) + a) * (2 ** 24 - 1)))
        # Wait for RC to settle
        time.sleep(0.01)

        # Averaging
        avg = []
        for j in range(d):
            avg.append(read_buffer())
        
        # along the axis 0
        data = np.average(avg, 0)

        # Decimate
        decim = data.shape[1]
        # I think this value was empirically determined based on 18 ps per element.
        # Oscilloscope.py has a 17E-12 somewhere to give it a bit of headroom, which also needs to be adjusted if your FPGA is different.
        # If your FPGA is not a LFE5UM5G-85F, then this value might be different (especially for LFE5UM and LFE5U), maybe up to 50 ps.
        # For example if it is 50 ps, then in Oscilloscope.py delay = 45E-12 and here decim = 67.
        # Though on the other hand, you will probably need to lower the clock frequency of the sampler so that would make decim larger again.
        decim = 186 # 1 / sample_freq / delay

        reduced = data[:, 5:5+decim].flatten()
        acquisitions.append(reduced)
        #plt.plot(np.linspace(0, 18e-12 * len(reduced), len(reduced), endpoint = False), reduced + i * 1.1)
    
    print(f"Acquisition time: {(time.clock_gettime_ns(0) - start) / 1e9:.3f}s")

    ### Plotting
    
    samples = np.asarray(acquisitions)

    # Various filter functions
    #sinc = lambda x : np.choose(x == 0, [np.sin(2 * np.pi * x) / (2 * np.pi * x), 1])
    gaussian = lambda x : np.exp(-x ** 2)

    # Calculate the convolution kernel
    kernel = np.zeros([11, 121])
    for i in range(u_blur_n * 2 + 1):
        for j in range(t_blur_n * 2 + 1):
            kernel[i, j] = gaussian((i - u_blur_n) / 2) * gaussian((j - t_blur_n) / 5)
    
    kernel /= kernel.sum()
    samples = scipy.signal.convolve(samples, kernel, mode='same')[u_blur_n:]

    #for i in range(samples.shape[0]):
    #    samples[i] = np.convolve(samples[i], gaussian(np.linspace(-6, 6, 121)), mode='same')
#
    #for i in range(samples.shape[1]):
    #    samples[:, i] = np.convolve(samples[:, i], gaussian(np.linspace(-2.5, 2.5, 11)), mode='same')


    # Analog oscilloscope colors (maybe "#001F00" instead of "#000000")
    colors = ["#000000", "#00FF00"]
    scopemap = LinearSegmentedColormap.from_list("mycmap", colors)

    # Use this instead of 1 to get the length in meters, assuming a velocity factor of 0.7
    factor = 0.3 / 2 * 0.7
    factor = 1

    #plt.imshow(np.asarray(acquisitions)[:, :1000], interpolation='nearest', aspect='auto')
    if False: # True to show raw data
        plt.imshow(samples[::-1], interpolation='nearest', aspect='auto', cmap="Greys", extent=[0, 18e-3 * (samples.shape[1] - 1) * factor, a, b])
        plt.xlabel("d (m)")
        plt.show()


    samples = np.log1p(np.abs(samples[1:, :] - samples[:-1, :]))
    #trace = np.average(samples / np.average(samples, 0), 0, weights = [i for i in range(samples.shape[0])]) * (b - a) + a # This does work but is a bit offset for some reason
    # Array with weights ranging from a to b, like [[a, ..., b], ..., [a, ..., b]] and then transposes it to [[a, ..., a], ..., [b, ..., b]]
    vals = (np.ones(samples.shape[::-1]) * np.linspace(a, b, samples.shape[0])).T
    # Weighted average to get one voltage per time position at which the real voltage most likely is
    trace = np.average(vals, 0, weights = samples)

    plt.imshow(samples[::-1], interpolation='nearest', aspect='auto', cmap=scopemap, extent=[0, 18e-3 * (samples.shape[1] - 1) * factor, a, b])
    plt.plot(np.linspace(0, 18e-3 * (samples.shape[1] - 1) * factor, samples.shape[1]), trace, color="#FF0000")
    plt.xlabel("t (ns)")
    
    #ax2 = ax1.twiny()
    #ax2.set_xticks(ax1.get_xticks())
    #ax2.set_xbound(ax1.get_xbound())
    #ax2.set_xticklabels([f"{x / (0.3 / 2 * 0.7):.1f}" for x in ax1.get_xticks()])


    #plt.plot(np.linspace(0, 18e-3 * (samples.shape[1] - 1) * 0.3 / 2 * 0.7, samples.shape[1]), trace, color="#FF0000")
    plt.show()



# I think uncommenting this yields a logic analyzer which also displays the frequency

#data = read_buffer()
#plt.plot(data[:10, ].T)

#cleaned = reduced[:-1] | reduced[1:]
#
#for i in range(10):
#    cleaned = cleaned[:-1] | cleaned[1:]
#
##plt.plot(np.linspace(0, 18e-12 * len(reduced), len(reduced), endpoint = False)[5:-6], cleaned)
#
#rises = (cleaned[:-1] ^ cleaned[1:]) & cleaned[1:]
##plt.plot(np.linspace(0, 18e-12 * len(reduced), len(reduced), endpoint = False)[6:-6], rises)
#print("Frequency:", np.sum(rises) / (18E-6 * len(reduced)), "MHz")

#for i in range(data.shape[0]):
#    plt.plot(np.asarray([1, 1]) * i * decim * 18E-12, [-0.1, 1.1], '-k')


# Reset voltages
write_register(0, 0x0)
write_register(1, 0x0)

# Acquire and plot the data
acquire()
