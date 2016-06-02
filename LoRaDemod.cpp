// Copyright (c) 2016-2016 Lime Microsystems
// SPDX-License-Identifier: BSL-1.0

#include <Pothos/Framework.hpp>
#include <iostream>
#include <complex>
#include <cmath>
#include "LoRaDetector.hpp"

/***********************************************************************
 * |PothosDoc Lora Demod
 *
 * Demodulate LoRa packets from a complex sample stream into symbols.
 *
 * <h2>Input format</h2>
 *
 * The input sample stream should be down converted from the specified
 * carrier frequency and at the specified LoRa BW parameter.
 *
 * <h2>Output format</h2>
 *
 * The output port 0 produces a packet containing demodulated symbols.
 * The format of the packet payload is a buffer of unsigned shorts.
 * A 16-bit short can fit all size symbols from 7 to 12 bits.
 *
 * <h2>Debug port raw</h2>
 *
 * The raw debug port outputs the LoRa signal annotated with labels
 * for important synchronization points in the input sample stream.
 *
 * <h2>Debug port dec</h2>
 *
 * The dec debug port outputs the LoRa signal downconverted
 * by a locally generated chirp with the same annotation labels as the raw output.
 *
 * |category /Demod
 * |keywords lora
 *
 * |param sf[Spread factor] The spreading factor controls the symbol spread.
 * Each symbol will occupy 2^SF number of samples given the waveform BW.
 * |default 8
 * |option 7
 * |option 8
 * |option 9
 * |option 10
 * |option 11
 * |option 12
 *
 * |param sync[Sync word] The sync word is a 2-nibble, 2-symbol sync value.
 * The sync word is encoded after the up-chirps and before the down-chirps.
 * The demodulator ignores packets that do not match the sync word.
 * |default 0x12
 *
 * |param mtu[Symbol MTU] Produce MTU symbols after sync is found.
 * The demodulator does not inspect the payload and will simply
 * produce the specified number of symbols once synchronized.
 * |default 256
 *
 * |factory /lora/lora_demod(sf)
 * |setter setSync(sync)
 * |setter setMTU(mtu)
 **********************************************************************/
class LoRaDemod : public Pothos::Block
{
public:
    LoRaDemod(const size_t sf):
        N(1 << sf),
        _detector(N),
        _sync(0),
        _mtu(256)
    {
        this->registerCall(this, POTHOS_FCN_TUPLE(LoRaDemod, setSync));
        this->registerCall(this, POTHOS_FCN_TUPLE(LoRaDemod, setMTU));
        this->setupInput(0, typeid(std::complex<float>));
        this->setupOutput(0);
        this->setupOutput("raw", typeid(std::complex<float>));
        this->setupOutput("dec", typeid(std::complex<float>));

        //use at most two input symbols available
        this->input(0)->setReserve(N*2);

        //store port pointers to avoid lookup by name
        _rawPort = this->output("raw");
        _decPort = this->output("dec");

        //generate chirp table
        double phaseAccum = 0.0;
        for (size_t i = 0; i < N; i++)
        {
            double phase = (2*(i+N/2)*M_PI)/N;
            phaseAccum += phase;
            auto entry = std::polar(1.0, phaseAccum);
            entry = std::conj(entry); //avoids conjugate later
            _chirpTable.push_back(std::complex<float>(entry));
        }
    }

    static Block *make(const size_t sf)
    {
        return new LoRaDemod(sf);
    }

    void setSync(const unsigned char sync)
    {
        _sync = sync;
    }

    void setMTU(const size_t mtu)
    {
        _mtu = mtu;
    }

    void activate(void)
    {
    }

    void work(void)
    {
        auto inPort = this->input(0);
        if (inPort->elements() < N*2) return;
        if (_rawPort->elements() < N*2) return;
        if (_decPort->elements() < N*2) return;

        auto inBuff = inPort->buffer().as<const std::complex<float> *>();
        auto rawBuff = _rawPort->buffer().as<std::complex<float> *>();
        auto decBuff = _decPort->buffer().as<std::complex<float> *>();

        for (size_t i = 0; i < N; i++)
        {
            auto samp = inBuff[i];
            auto decd = samp*_chirpTable[i];
            rawBuff[i] = samp;
            decBuff[i] = decd;
            _detector.feed(i, decd);
        }

        auto freqError = _detector.detect();

        inPort->consume(N-freqError);
        _rawPort->produce(N-freqError);
        _decPort->produce(N-freqError);
        _decPort->postLabel(Pothos::Label("x", Pothos::Object(), 0));
    }

private:
    //configuration
    const size_t N;
    LoRaDetector<float> _detector;
    std::vector<std::complex<float>> _chirpTable;
    unsigned char _sync;
    size_t _mtu;
    Pothos::OutputPort *_rawPort;
    Pothos::OutputPort *_decPort;

    //state
};

static Pothos::BlockRegistry registerLoRaDemod(
    "/lora/lora_demod", &LoRaDemod::make);
