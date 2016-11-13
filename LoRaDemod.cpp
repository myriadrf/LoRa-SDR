// Copyright (c) 2016-2016 Lime Microsystems
// SPDX-License-Identifier: BSL-1.0

#include <Pothos/Framework.hpp>
#include <iostream>
#include <complex>
#include <cstring>
#include <cmath>
#include "LoRaDetector.hpp"

/***********************************************************************
 * |PothosDoc LoRa Demod
 *
 * Demodulate LoRa packets from a complex sample stream into symbols.
 *
 * <h2>Input format</h2>
 *
 * The input port 0 accepts a complex sample stream of modulated chirps
 * received at the specified bandwidth and carrier frequency.
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
 * |category /LoRa
 * |keywords lora
 *
 * |param sf[Spread factor] The spreading factor controls the symbol spread.
 * Each symbol will occupy 2^SF number of samples given the waveform BW.
 * |default 10
 *
 * |param sync[Sync word] The sync word is a 2-nibble, 2-symbol sync value.
 * The sync word is encoded after the up-chirps and before the down-chirps.
 * The demodulator ignores packets that do not match the sync word.
 * |default 0x12
 *
 * |param thresh[Threshold] The minimum required level in dB for the detector.
 * The threshold level is used to enter and exit the demodulation state machine.
 * |units dB
 * |default -30.0
 *
 * |param mtu[Symbol MTU] Produce MTU at most symbols after sync is found.
 * The demodulator does not inspect the payload and will produce at most
 * the specified MTU number of symbols or less if the detector squelches.
 * |units symbols
 * |default 256
 *
 * |factory /lora/lora_demod(sf)
 * |setter setSync(sync)
 * |setter setThreshold(thresh)
 * |setter setMTU(mtu)
 **********************************************************************/
class LoRaDemod : public Pothos::Block
{
public:
    LoRaDemod(const size_t sf):
        N(1 << sf),
        _fineSteps(128),
        _detector(N),
        _sync(0x12),
        _thresh(-30.0),
        _mtu(256)
    {
        this->registerCall(this, POTHOS_FCN_TUPLE(LoRaDemod, setSync));
        this->registerCall(this, POTHOS_FCN_TUPLE(LoRaDemod, setThreshold));
        this->registerCall(this, POTHOS_FCN_TUPLE(LoRaDemod, setMTU));
        this->setupInput(0, typeid(std::complex<float>));
        this->setupOutput(0);
        this->setupOutput("raw", typeid(std::complex<float>));
        this->setupOutput("dec", typeid(std::complex<float>));
        this->setupOutput("fft", typeid(std::complex<float>));
        
        this->registerSignal("error");
        this->registerSignal("power");
        this->registerSignal("snr");

        //use at most two input symbols available
        this->input(0)->setReserve(N*2);

        //store port pointers to avoid lookup by name
        _rawPort = this->output("raw");
        _decPort = this->output("dec");
        _fftPort = this->output("fft");
        
        //generate chirp table
        float phase = -M_PI;
        double phaseAccum = 0.0;
        for (size_t i = 0; i < N; i++)
        {
            phaseAccum += phase;
            auto entry = std::polar(1.0, phaseAccum);
            _upChirpTable.push_back(std::complex<float>(std::conj(entry)));
            _downChirpTable.push_back(std::complex<float>(entry));
            phase += (2*M_PI)/N;
        }
        phaseAccum = 0.0;
        phase = 2.0 * M_PI / (N * _fineSteps);
        for (size_t i = 0; i < N * _fineSteps; i++){
            phaseAccum += phase;
            auto entry = std::polar(1.0, phaseAccum);
            _fineTuneTable.push_back(std::complex<float>(entry));
        }
        
        _fineTuneIndex = 0;
    }

    static Block *make(const size_t sf)
    {
        return new LoRaDemod(sf);
    }

    void setSync(const unsigned char sync)
    {
        _sync = sync;
    }

    void setThreshold(const double thresh_dB)
    {
        _thresh = thresh_dB;
    }

    void setMTU(const size_t mtu)
    {
        _mtu = mtu;
    }

    void activate(void)
    {
        _state = STATE_FRAMESYNC;
        _chirpTable = _upChirpTable.data();
    }

    void work(void)
    {
        auto inPort = this->input(0);
        if (inPort->elements() < N*2) return;
        
        size_t total = 0;
        auto inBuff = inPort->buffer().as<const std::complex<float> *>();
        auto rawBuff = _rawPort->buffer().as<std::complex<float> *>();
        auto decBuff = _decPort->buffer().as<std::complex<float> *>();
        auto fftBuff = _fftPort->buffer().as<std::complex<float> *>();

        //process the available symbol
        for (size_t i = 0; i < N; i++){
            auto samp = inBuff[i];
            auto decd = samp*_chirpTable[i] * _fineTuneTable[_fineTuneIndex];
            _fineTuneIndex -= _finefreqError * _fineSteps;
            if (_fineTuneIndex < 0) _fineTuneIndex += N * _fineSteps;
            else if (_fineTuneIndex >= int(N * _fineSteps)) _fineTuneIndex -= N * _fineSteps;
            rawBuff[i] = samp;
            decBuff[i] = decd;
            _detector.feed(i, decd);
        }
        float power = 0;
        float powerAvg = 0;
        float snr = 0;
        float fIndex = 0;
        
        auto value = _detector.detect(power,powerAvg,fIndex,fftBuff);
        snr = power - powerAvg;
        const bool squelched = (snr < _thresh);

        switch (_state)
        {
        ////////////////////////////////////////////////////////////////
        case STATE_FRAMESYNC:
        ////////////////////////////////////////////////////////////////
        {
            //format as observed from inspecting RN2483
            bool syncd = not squelched and (_prevValue+4)/8 == 0;
            bool match0 = (value+4)/8 == unsigned(_sync>>4);
            bool match1 = false;

            //if the symbol matches sync word0 then check sync word1 as well
            //otherwise assume its the frame sync and adjust for frequency error
            if (syncd and match0)
            {
                int ft = _fineTuneIndex;
                for (size_t i = 0; i < N; i++)
                {
                    auto samp = inBuff[i + N];
                    auto decd = samp*_chirpTable[i] * _fineTuneTable[ft];
                    ft -= _finefreqError * _fineSteps;
                    if (ft < 0) ft += N * _fineSteps;
                    else if (ft >= int(N * _fineSteps)) ft -= N * _fineSteps;
                    rawBuff[i+N] = samp;
                    decBuff[i+N] = decd;
                    _detector.feed(i, decd);
                }
                auto value1 = _detector.detect(power,powerAvg,fIndex);
                //format as observed from inspecting RN2483
                match1 = (value1+4)/8 == unsigned(_sync & 0xf);
            }

            if (syncd and match0 and match1)
            {
                total = 2*N;
                _state = STATE_DOWNCHIRP0;
                _chirpTable = _downChirpTable.data();
                _id = "SYNC";
            }

            //otherwise its a frequency error
            else if (not squelched)
            {
                total = N - value;
                _finefreqError += fIndex;
				std::stringstream stream;
				stream.precision(4);
				stream << std::fixed << "P " << fIndex;
				_id = stream.str();
 //               _id = "P " + std::to_string(fIndex);
            }

            //just noise
            else
            {
                total = N;
                _finefreqError = 0;
                _fineTuneIndex = 0;
                _id = "";
            }

        } break;

        ////////////////////////////////////////////////////////////////
        case STATE_DOWNCHIRP0:
        ////////////////////////////////////////////////////////////////
        {
            _state = STATE_DOWNCHIRP1;
            total = N;
            _id = "DC";
            int error = value;
            if (value > N/2) error -= N;
            //std::cout << "error0 " << error << std::endl;
            _freqError = error;
        } break;

        ////////////////////////////////////////////////////////////////
        case STATE_DOWNCHIRP1:
        ////////////////////////////////////////////////////////////////
        {
            _state = STATE_QUARTERCHIRP;
            total = N;
            _chirpTable = _upChirpTable.data();
            _id = "";
            _outSymbols = Pothos::BufferChunk(typeid(int16_t), _mtu);

            int error = value;
            if (value > N/2) error -= N;
            //std::cout << "error1 " << error << std::endl;
            _freqError = (_freqError + error)/2;

            this->emitSignal("error", _freqError);
            this->emitSignal("power", power);
            this->emitSignal("snr", snr);
        } break;

        ////////////////////////////////////////////////////////////////
        case STATE_QUARTERCHIRP:
        ////////////////////////////////////////////////////////////////
        {
            _state = STATE_DATASYMBOLS;
            
            total = N/4 + (_freqError / 2);
            _finefreqError += (_freqError / 2);
            
            _symCount = 0;
            _id = "QC";
        } break;

        ////////////////////////////////////////////////////////////////
        case STATE_DATASYMBOLS:
        ////////////////////////////////////////////////////////////////
        {
            total = N;
            _outSymbols.as<int16_t *>()[_symCount++] = int16_t(value);
            if (_symCount >= _mtu or squelched)
            {
                //for (size_t j = 0; j < _symCount; j++)
                //    std::cout << "demod[" << j << "]=" << _outSymbols.as<const uint16_t *>()[j] << std::endl;
                Pothos::Packet pkt;
                pkt.payload = _outSymbols;
                pkt.payload.length = _symCount*sizeof(int16_t);
                this->output(0)->postMessage(pkt);
                _finefreqError = 0;
                _state = STATE_FRAMESYNC;
            }
			std::stringstream stream;
			stream.precision(4);
			stream << std::fixed << "S" << _symCount << " " << fIndex;
			_id = stream.str();
            //_id = "S" + std::to_string(_symCount) + " " + std::to_string(fIndex);
            
           // _finefreqError += fIndex;
            
        } break;

        }

        if (not _id.empty())
        {
            _rawPort->postLabel(Pothos::Label(_id, Pothos::Object(), 0));
            _decPort->postLabel(Pothos::Label(_id, Pothos::Object(), 0));
            _fftPort->postLabel(Pothos::Label(_id, Pothos::Object(), 0));
        }
        inPort->consume(total);
        _rawPort->produce(total);
        _decPort->produce(total);
        
        _fftPort->produce(N);
        
        _prevValue = value;
    }

    //! Custom output buffer manager with slabs large enough for debug output
    Pothos::BufferManager::Sptr getOutputBufferManager(const std::string &name, const std::string &domain)
    {
        if (name == "raw" or name == "dec")
        {
            this->output(name)->setReserve(N * 2);
            Pothos::BufferManagerArgs args;
            args.bufferSize = N*2*sizeof(std::complex<float>);
            return Pothos::BufferManager::make("generic", args);
        }else if (name == "fft"){
            this->output(name)->setReserve(N);
            Pothos::BufferManagerArgs args;
            args.bufferSize = N*sizeof(std::complex<float>);
            return Pothos::BufferManager::make("generic", args);
        }
        return Pothos::Block::getOutputBufferManager(name, domain);
    }

    //! Custom input buffer manager with slabs large enough for fft input
    Pothos::BufferManager::Sptr getInputBufferManager(const std::string &name, const std::string &domain)
    {
        if (name == "0")
        {
            Pothos::BufferManagerArgs args;
            args.bufferSize = std::max(args.bufferSize,
                              N*2*sizeof(std::complex<float>));
            return Pothos::BufferManager::make("generic", args);
        }
        return Pothos::Block::getInputBufferManager(name, domain);
    }

private:
    //configuration
    const size_t N;
    const size_t _fineSteps;
    LoRaDetector<float> _detector;
    std::complex<float> *_chirpTable;
    std::vector<std::complex<float>> _upChirpTable;
    std::vector<std::complex<float>> _downChirpTable;
    std::vector<std::complex<float>> _fineTuneTable;
    unsigned char _sync;
    float _thresh;
    size_t _mtu;
    Pothos::OutputPort *_rawPort;
    Pothos::OutputPort *_decPort;
    Pothos::OutputPort *_fftPort;

    //state
    enum LoraDemodState
    {
        STATE_FRAMESYNC,
        STATE_DOWNCHIRP0,
        STATE_DOWNCHIRP1,
        STATE_QUARTERCHIRP,
        STATE_DATASYMBOLS,
    };
    LoraDemodState _state;
    size_t _symCount;
    Pothos::BufferChunk _outSymbols;
    std::string _id;
    short _prevValue;
    int _freqError;
    int _fineTuneIndex;
    float _finefreqError;
};

static Pothos::BlockRegistry registerLoRaDemod(
    "/lora/lora_demod", &LoRaDemod::make);
