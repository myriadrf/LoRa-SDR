#!/usr/bin/env python

import sys
import math
import cmath
import wave
import scipy.signal
import numpy as np
import matplotlib.pyplot as plt
import SoapySDR
from SoapySDR import *
import RN2483

SDR_ARGS = 'driver=rtlsdr'
SDR_RATE = 2*1024e3
SDR_THRESH = 0.01
TTY_PORT = '/dev/ttyACM0'
FREQ = 868.1e6
BW = 250e3
SF = 8
RDD = 4
SYNC = 0x83

def transmitAndCollect(rn2483, sdr, rxStream, payload):
    """
    Transmit the specified payload over the RN2483
    and receive through the RTLSDR device.
    \return a numpy array with LoRa samples
    """
    buff = np.array([0]*1024, np.complex64)

    #flush
    while True:
        sr = sdr.readStream(rxStream, [buff], len(buff))
        if sr.ret == SOAPY_SDR_TIMEOUT: break

    sdr.activateStream(rxStream) #start streaming

    for i in range(16):
        sdr.readStream(rxStream, [buff], len(buff))

    rn2483.transmit(payload)

    #receive some samples
    loraSamples = np.array([])
    while True:
        sr = sdr.readStream(rxStream, [buff], len(buff))
        assert(sr.ret > 0)
        found = np.std(buff) > SDR_THRESH
        if not found and not loraSamples.size: continue
        loraSamples = np.concatenate((loraSamples, buff))
        if not found: break

    sdr.deactivateStream(rxStream) #stop streaming

    return loraSamples

if __name__ == '__main__':

    ####################################################################
    #connect to devices
    rn2483 = RN2483.RN2483(TTY_PORT)
    rn2483.configLoRa(freq=FREQ,
        bw=int(BW/1e3),
        crc='off',
        cr='4/%d'%(RDD + 4),
        sf='sf%d'%SF,
        sync=SYNC)

    sdr = SoapySDR.Device(SDR_ARGS)
    sdr.setFrequency(SOAPY_SDR_RX, 0, FREQ)
    sdr.setSampleRate(SOAPY_SDR_RX, 0, SDR_RATE)
    rxStream = sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32)

    ####################################################################
    #collect data and plot
    loraSamples = transmitAndCollect(rn2483, sdr, rxStream, ''.join([chr(0xff)]*8))
    print("Found %d samples"%loraSamples.size)

    loraSamples = loraSamples[15000:25000] #just a few for plotting purposes
    resampled = scipy.signal.resample(loraSamples, int(BW*loraSamples.size/SDR_RATE))
    fmdemodResamp = np.angle(resampled[1:] * np.conj(resampled)[:-1])

    fig = plt.figure(figsize=(40, 10))
    fig.suptitle('Extracted samples', fontsize=12)

    ax = fig.add_subplot(1, 1, 1)
    ax.set_title('Raw content')
    ax.plot(np.arange(0, resampled.size, 1), np.real(resampled))
    ax.plot(np.arange(0, resampled.size, 1), np.imag(resampled))
    ax.plot(np.arange(0, fmdemodResamp.size, 1), fmdemodResamp)
    ax.grid(True)

    outPath = '/tmp/out.png'
    print("Writing plot to %s"%outPath)
    fig.savefig(outPath)
    plt.close(fig)

    sdr.closeStream(rxStream)
    print("Done!")
    exit(0)
