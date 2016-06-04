// Copyright (c) 2016-2016 Lime Microsystems
// SPDX-License-Identifier: BSL-1.0

#include <Pothos/Framework.hpp>
#include <iostream>
#include <complex>
#include <cmath>
#include "LoRaDetector.hpp"

/***********************************************************************
 * |PothosDoc LoRa Mod
 *
 * Modulate LoRa packets from symbols into a complex sample stream.
 *
 * <h2>Input format</h2>>
 *
 * The input port 0 accepts a packet containing pre-modulated symbols.
 * The format of the packet payload is a buffer of unsigned shorts.
 * A 16-bit short can fit all size symbols from 7 to 12 bits.
 *
 * <h2>Output format</h2>
 *
 * The output port 0 produces a complex sample stream of modulated chirps
 * to be transmitted at the specified bandwidth and carrier frequency.
 *
 * |category /LoRa
 * |keywords lora
 *
 * |param sf[Spread factor] The spreading factor controls the symbol spread.
 * Each symbol will occupy 2^SF number of samples given the waveform BW.
 * |default 8
 *
 * |param sync[Sync word] The sync word is a 2-nibble, 2-symbol sync value.
 * The sync word is encoded after the up-chirps and before the down-chirps.
 * |default 0x12
 *
 * |param padding[Padding] Pad out the end of a packet with zeros.
 * This is mostly useful for simulation purposes, though some padding
 * may be desirable to flush samples through the radio transmitter.
 * |units symbols
 * |default 1
 *
 * |param ampl[Amplitude] The digital transmit amplitude.
 * |default 0.3
 *
 * |factory /lora/lora_mod(sf)
 * |setter setSync(sync)
 * |setter setPadding(padding)
 * |setter setAmplitude(ampl)
 **********************************************************************/
class LoRaMod : public Pothos::Block
{
public:
    LoRaMod(const size_t sf):
        N(1 << sf),
        _sync(0x12),
        _padding(1),
        _ampl(0.3f)
    {
        this->registerCall(this, POTHOS_FCN_TUPLE(LoRaMod, setSync));
        this->registerCall(this, POTHOS_FCN_TUPLE(LoRaMod, setPadding));
        this->registerCall(this, POTHOS_FCN_TUPLE(LoRaMod, setAmplitude));
        this->setupInput(0);
        this->setupOutput(0, typeid(std::complex<float>));

        float phase = -M_PI;
        for (size_t i = 0; i < N; i++)
        {
            _upChirpPhase.push_back(phase);
            phase += (2*M_PI)/N;
        }
    }

    static Block *make(const size_t sf)
    {
        return new LoRaMod(sf);
    }

    void setSync(const unsigned char sync)
    {
        _sync = sync;
    }

    void setPadding(const size_t padding)
    {
        _padding = padding;
    }

    void setAmplitude(const float ampl)
    {
        _ampl = ampl;
    }

    void activate(void)
    {
        _state = STATE_WAITINPUT;
    }

    void work(void)
    {
        auto outPort = this->output(0);
        if (outPort->elements() < N)
        {
            outPort->popBuffer(outPort->elements());
            return;
        }
        auto samps = outPort->buffer().as<std::complex<float> *>();
        size_t i = 0;

        //std::cout << "mod state " << int(_state) << std::endl;
        switch (_state)
        {
        ////////////////////////////////////////////////////////////////
        case STATE_WAITINPUT:
        ////////////////////////////////////////////////////////////////
        {
            if (not this->input(0)->hasMessage()) return;
            auto msg = this->input(0)->popMessage();
            auto pkt = msg.extract<Pothos::Packet>();
            _payload = pkt.payload;
            _state = STATE_FRAMESYNC;
            _counter = 10;
            _phaseAccum = 0;
            _id = "";
        } break;

        ////////////////////////////////////////////////////////////////
        case STATE_FRAMESYNC:
        ////////////////////////////////////////////////////////////////
        {
            _counter--;
            for (i = 0; i < N; i++)
            {
                _phaseAccum += _upChirpPhase[i];
                samps[i] = std::polar(_ampl, _phaseAccum);
            }
            if (_counter == 0) _state = STATE_SYNCWORD0;
            _id = "X";
        } break;

        ////////////////////////////////////////////////////////////////
        case STATE_SYNCWORD0:
        ////////////////////////////////////////////////////////////////
        {
            const int sw0 = (_sync >> 4)*8;
            const float freq = (2*M_PI*sw0)/N;
            for (i = 0; i < N; i++)
            {
                _phaseAccum += _upChirpPhase[i] + freq;
                samps[i] = std::polar(_ampl, _phaseAccum);
            }
            _state = STATE_SYNCWORD1;
            _id = "SYNC";
        } break;

        ////////////////////////////////////////////////////////////////
        case STATE_SYNCWORD1:
        ////////////////////////////////////////////////////////////////
        {
            const int sw1 = (_sync & 0xf)*8;
            const float freq = (2*M_PI*sw1)/N;
            for (i = 0; i < N; i++)
            {
                _phaseAccum += _upChirpPhase[i] + freq;
                samps[i] = std::polar(_ampl, _phaseAccum);
            }
            _state = STATE_DOWNCHIRP0;
            _id = "";
        } break;

        ////////////////////////////////////////////////////////////////
        case STATE_DOWNCHIRP0:
        ////////////////////////////////////////////////////////////////
        {
            for (i = 0; i < N; i++)
            {
                _phaseAccum += _upChirpPhase[i];
                samps[i] = std::polar(_ampl, -_phaseAccum);
            }
            _state = STATE_DOWNCHIRP1;
            _id = "DC";
        } break;

        ////////////////////////////////////////////////////////////////
        case STATE_DOWNCHIRP1:
        ////////////////////////////////////////////////////////////////
        {
            for (i = 0; i < N; i++)
            {
                _phaseAccum += _upChirpPhase[i];
                samps[i] = std::polar(_ampl, -_phaseAccum);
            }
            _state = STATE_QUARTERCHIRP;
            _id = "";
        } break;

        ////////////////////////////////////////////////////////////////
        case STATE_QUARTERCHIRP:
        ////////////////////////////////////////////////////////////////
        {
            for (i = 0; i < N/4; i++)
            {
                _phaseAccum += _upChirpPhase[i];
                samps[i] = std::polar(_ampl, -_phaseAccum);
            }
            _state = STATE_DATASYMBOLS;
            _counter = 0;
            _id = "QC";
        } break;

        ////////////////////////////////////////////////////////////////
        case STATE_DATASYMBOLS:
        ////////////////////////////////////////////////////////////////
        {
            const int sym = _payload.as<const uint16_t *>()[_counter++];
            const float freq = (2*M_PI*sym)/N;

            for (i = 0; i < N; i++)
            {
                _phaseAccum += _upChirpPhase[i] + freq;
                samps[i] = std::polar(_ampl, _phaseAccum);
            }
            if (_counter >= _payload.elements())
            {
                _state = STATE_PADSYMBOLS;
                _counter = 0;
            }
            _id = "S";
        } break;

        ////////////////////////////////////////////////////////////////
        case STATE_PADSYMBOLS:
        ////////////////////////////////////////////////////////////////
        {
            _counter++;
            for (i = 0; i < N; i++) samps[i] = 0.0f;
            if (_counter >= _padding)
            {
                _state = STATE_WAITINPUT;
                outPort->postLabel(Pothos::Label("txEnd", Pothos::Object(), N-1));
            }
            _id = "";
        } break;

        }

        //keep phase in range
        while (_phaseAccum > 2*M_PI) _phaseAccum -= 2*M_PI;

        if (not _id.empty())
        {
            outPort->postLabel(Pothos::Label(_id, Pothos::Object(), 0));
        }
        outPort->produce(i);
    }

    //! Custom output buffer manager with slabs large enough for output chirp
    Pothos::BufferManager::Sptr getOutputBufferManager(const std::string &name, const std::string &domain)
    {
        if (name == "0")
        {
            Pothos::BufferManagerArgs args;
            args.bufferSize = N*sizeof(std::complex<float>);
            return Pothos::BufferManager::make("generic", args);
        }
        return Pothos::Block::getOutputBufferManager(name, domain);
    }

private:
    //configuration
    const size_t N;
    unsigned char _sync;
    size_t _padding;
    float _ampl;

    //state
    enum LoraDemodState
    {
        STATE_WAITINPUT,
        STATE_FRAMESYNC,
        STATE_SYNCWORD0,
        STATE_SYNCWORD1,
        STATE_DOWNCHIRP0,
        STATE_DOWNCHIRP1,
        STATE_QUARTERCHIRP,
        STATE_DATASYMBOLS,
        STATE_PADSYMBOLS,
    };
    LoraDemodState _state;
    size_t _counter;
    Pothos::BufferChunk _payload;
    std::vector<float> _upChirpPhase;
    float _phaseAccum;
    std::string _id;
};

static Pothos::BlockRegistry registerLoRaMod(
    "/lora/lora_mod", &LoRaMod::make);
