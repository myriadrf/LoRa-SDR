// Copyright (c) 2016-2016 Lime Microsystems
// SPDX-License-Identifier: BSL-1.0

#include <Pothos/Framework.hpp>
#include <iostream>
#include <cstring>
#include "LoRaCodes.hpp"

/***********************************************************************
 * |PothosDoc LoRa Decoder
 *
 * Decode LoRa modulation symbols into output bytes.
 * This is a simple decoder and does not offer many options.
 * The job of the decoder is to gray encode to convert measurement error
 * into bit errors, deinterleave, handle error correction, and descramble.
 *
 * <h2>Input format</h2>
 *
 * A packet message with a payload containing LoRa modulation symbols.
 * The format of the packet payload is a buffer of unsigned shorts.
 * A 16-bit short can fit all size symbols from 7 to 12 bits.
 *
 * <h2>Output format</h2>
 *
 * A packet message with a payload containing bytes received.
 *
 * |category /LoRa
 * |keywords lora
 *
 * |param sf[Spread factor] The spreading factor sets the bits per symbol.
 * |default 10
 *
 * |param ppm[Symbol size] The size of the symbol set (_ppm <= SF).
 * Specify _ppm less than the spread factor to use a reduced symbol set.
 * |default 8
 *
 * |param cr[Coding Rate] The number of error correction bits.
 * |option [4/4] "4/4"
 * |option [4/5] "4/5"
 * |option [4/6] "4/6"
 * |option [4/7] "4/7"
 * |option [4/8] "4/8"
 * |default "4/8"
 *
 * |param whitening Enable/disable whitening of the decoded message.
 * |option [On] true
 * |option [Off] false
 * |default true
 *
 * |factory /lora/lora_decoder()
 * |setter setSpreadFactor(sf)
 * |setter setSymbolSize(ppm)
 * |setter setCodingRate(cr)
 * |setter enableWhitening(whitening)
 **********************************************************************/
class LoRaDecoder : public Pothos::Block
{
public:
    LoRaDecoder(void):
        _sf(10),
        _ppm(8),
        _rdd(4),
        _whitening(true),
        _dropped(0)
    {
        this->registerCall(this, POTHOS_FCN_TUPLE(LoRaDecoder, setSpreadFactor));
        this->registerCall(this, POTHOS_FCN_TUPLE(LoRaDecoder, setSymbolSize));
        this->registerCall(this, POTHOS_FCN_TUPLE(LoRaDecoder, setCodingRate));
        this->registerCall(this, POTHOS_FCN_TUPLE(LoRaDecoder, enableWhitening));
        this->registerCall(this, POTHOS_FCN_TUPLE(LoRaDecoder, dropped));
        this->registerProbe("dropped");
        this->setupInput("0");
        this->setupOutput("0");
    }

    static Block *make(void)
    {
        return new LoRaDecoder();
    }

    void setSpreadFactor(const size_t sf)
    {
        _sf = sf;
    }

    void setSymbolSize(const size_t ppm)
    {
        _ppm = ppm;
    }

    void setCodingRate(const std::string &cr)
    {
        if (cr == "4/4") _rdd = 0;
        //else if (cr == "4/5") _rdd = 1;
        //else if (cr == "4/6") _rdd = 2;
        else if (cr == "4/7") _rdd = 3;
        else if (cr == "4/8") _rdd = 4;
        else throw Pothos::InvalidArgumentException("LoRaDecoder::setCodingRate("+cr+")", "unknown coding rate");
    }

    void enableWhitening(const bool whitening)
    {
        _whitening = whitening;
    }

    unsigned long long dropped(void) const
    {
        return _dropped;
    }

    void activate(void)
    {
        _dropped = 0;
    }

    void work(void)
    {
        auto inPort = this->input(0);
        auto outPort = this->output(0);
        if (not inPort->hasMessage()) return;

        if (_ppm > _sf) throw Pothos::Exception("LoRaDecoder::work()", "failed check: PPM <= SF");

        //extract the input symbols
        auto msg = inPort->popMessage();
        auto pkt = msg.extract<Pothos::Packet>();
        const size_t numSymbols = roundUp(pkt.payload.elements(), 4 + _rdd);
        const size_t numCodewords = (numSymbols/(4 + _rdd))*_ppm;
        std::vector<uint16_t> symbols(numSymbols);
        std::memcpy(symbols.data(), pkt.payload.as<const void *>(), pkt.payload.length);

        //gray encode, when SF > _ppm, depad the LSBs with rounding
        for (auto &sym : symbols)
        {
            sym += (1 << (_sf-_ppm))/2; //increment by 1/2
            sym >>= (_sf-_ppm); //down shift to _ppm bits
            sym = binaryToGray16(sym);
        }

        //deinterleave the symbols into codewords
        std::vector<uint8_t> codewords(numCodewords);
        diagonalDeterleave(symbols.data(), numSymbols, codewords.data(), _ppm, _rdd);

        //decode each codeword as 2 bytes with correction
        bool error = false;
        std::vector<uint8_t> bytes(codewords.size()/2);
        if (_rdd == 0) for (size_t i = 0; i < bytes.size(); i++)
        {
            bytes[i] = codewords[i*2+0] << 4;
            bytes[i] |= codewords[i*2+1] & 0xf;
        }
        else if (_rdd == 3) for (size_t i = 0; i < bytes.size(); i++)
        {
            bytes[i] = decodeHamming74(codewords[i*2+0]) << 4;
            bytes[i] |= decodeHamming74(codewords[i*2+1]) & 0xf;
        }
        else if (_rdd == 4) for (size_t i = 0; i < bytes.size(); i++)
        {
            bytes[i] = decodeHamming84(codewords[i*2+0], error) << 4;
            bytes[i] |= decodeHamming84(codewords[i*2+1], error) & 0xf;
        }
        //if (error) return;

        //undo the whitening
        if (_whitening)
            SX1232RadioComputeWhitening(bytes.data(), bytes.size());

        //header and crc
        size_t length = bytes[0];
        if (length < 2) return this->drop();
        if (length > bytes.size()) return this->drop();
        uint8_t crc = checksum8(bytes.data(), length-1);
        if (crc != bytes[length-1]) return this->drop();

        //post the output bytes
        Pothos::Packet out;
        out.payload = Pothos::BufferChunk(typeid(uint8_t), length-2);
        std::memcpy(out.payload.as<void *>(), bytes.data()+1, out.payload.length);
        outPort->postMessage(out);
    }

private:

    void drop(void)
    {
        _dropped++;
    }

    size_t _sf;
    size_t _ppm;
    size_t _rdd;
    bool _whitening;
    unsigned long long _dropped;
};

static Pothos::BlockRegistry registerLoRaDecoder(
    "/lora/lora_decoder", &LoRaDecoder::make);
