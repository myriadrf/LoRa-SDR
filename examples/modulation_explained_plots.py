import os
import sys
import math
import cmath
import scipy.signal
import numpy as np
import matplotlib.pyplot as plt

def modulate(N, sym=0, numSamps=None):
    if numSamps is None: numSamps = N
    phase = -math.pi
    samps = list()
    accum = 0
    off = (2*math.pi*sym)/N
    for i in range(numSamps):
        accum += phase + off
        samps.append(cmath.rect(1.0, accum))
        phase += (2*math.pi)/N
    return np.array(samps)

if __name__ == '__main__':

    ####################################################################
    ## showing modulation basics
    ####################################################################
    SF = 8
    N = 1 << SF
    syms = [0, 0, 50, 100, 200]
    chirps = np.concatenate([modulate(N, sym) for sym in syms])
    fmdemod = np.angle(chirps[1:] * np.conj(chirps)[:-1])
    clock = np.concatenate([[3]+[0]*((1<<SF)-1) for i in range(len(syms))])

    fig = plt.figure(figsize=(20, 5))

    ax = fig.add_subplot(2, 1, 1)
    ax.set_title('Raw modulated I and Q: SF = %d, symbols=%s'%(SF, syms))
    ax.plot(np.arange(0, chirps.size, 1), np.real(chirps))
    ax.plot(np.arange(0, chirps.size, 1), np.imag(chirps))
    ax.grid(True)
    ax.set_xlim(0, chirps.size)

    ax = fig.add_subplot(2, 1, 2)
    ax.set_title('Frequency demodulated: SF = %d, symbols=%s'%(SF, syms))
    ax.plot(np.arange(0, fmdemod.size, 1), fmdemod)
    ax.plot(np.arange(0, len(syms)*(N), N), [0]*len(syms), 'xk', markersize=20)
    ax.grid(True)
    ax.set_xlim(0, chirps.size)

    outPath = '/tmp/plot0_mod.png'
    print("Writing plot to %s"%outPath)
    plt.tight_layout()
    fig.savefig(outPath)
    plt.close(fig)
    os.system('convert %s -trim -bordercolor white -border 5 %s'%(outPath, outPath))

    ####################################################################
    ## showing demodulation basics
    ####################################################################

    downchirp = np.conj(modulate(N))
    chirps = np.concatenate([modulate(N)[N/4:], chirps])
    dechirped = np.array([])
    i = 0
    outsyms = list()
    clock = list()
    ffts = list()
    while i <= chirps.size-(N):
        #print i
        chunk = chirps[i:i+(1 <<SF)] * downchirp
        value = np.argmax(np.abs(np.fft.fft(chunk)))
        outsyms.append(value)
        ffts.append(np.abs(np.fft.fft(chunk)))
        #print "val", value
        total = N
        #use this as the offset
        if i == 0: total -= value
        dechirped = np.concatenate([dechirped, chunk[:total]])
        clock.append(i)
        i += total
    fmdemod = np.angle(dechirped[1:] * np.conj(dechirped)[:-1])

    fig = plt.figure(figsize=(20, 5))

    ax = fig.add_subplot(2, 1, 1)
    ax.set_title('De-chirped I and Q: SF = %d'%(SF,))
    ax.plot(np.arange(0, dechirped.size, 1), np.real(dechirped))
    ax.plot(np.arange(0, dechirped.size, 1), np.imag(dechirped))
    ax.grid(True)
    ax.set_xlim(0, dechirped.size)

    ax = fig.add_subplot(2, 1, 2)
    ax.set_title('Frequency demodulated: SF = %d'%(SF,))
    ax.plot(np.arange(0, fmdemod.size, 1), fmdemod)
    ax.plot(clock, [0]*len(clock), 'xk', markersize=20)
    ax.grid(True)
    ax.set_xlim(0, dechirped.size)

    outPath = '/tmp/plot1_demod.png'
    print("Writing plot to %s"%outPath)
    plt.tight_layout()
    fig.savefig(outPath)
    plt.close(fig)
    os.system('convert %s -trim -bordercolor white -border 5 %s'%(outPath, outPath))

    fig = plt.figure(figsize=(20, 5))

    for i, fft in enumerate(ffts):
        ax = fig.add_subplot(2, 3, i+1)
        ax.set_title('FFT for symbol #%d, detected %d'%(i,outsyms[i]), fontsize=12)
        ax.plot(np.arange(0, fft.size, 1), fft)
        ax.plot([outsyms[i]], [fft[outsyms[i]]], 'xk', markersize=20)
        ax.grid(True)
        ax.get_yaxis().set_ticks([0, 100, 200, 300])
        #ax.tick_params(axis='both', which='major', labelsize=10)
        #ax.tick_params(axis='both', which='minor', labelsize=8)
        ax.set_xlim(0, N-1)
        ax.get_xaxis().set_ticks(np.arange(0, N, 64))

    outPath = '/tmp/plot2_ffts.png'
    print("Writing plot to %s"%outPath)
    plt.tight_layout()
    fig.savefig(outPath)
    plt.close(fig)
    os.system('convert %s -trim -bordercolor white -border 5 %s'%(outPath, outPath))
