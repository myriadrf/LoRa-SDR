// Copyright (c) 2016-2016 Lime Microsystems
// SPDX-License-Identifier: BSL-1.0

#include <Pothos/Framework.hpp>
#include <iostream>
#include <cstring>
#include "LoRaCodes.hpp"

/***********************************************************************
 * |PothosDoc LoRa Encoder
 *
 * Encode bytes into LoRa modulation symbols.
 * This is a simple encoder and does not offer many options.
 * The job of the encoder is to scramble, add error correction,
 * interleaving, and gray decoded to handle measurement error.
 *
 * <h2>Input format</h2>
 *
 * A packet message with a payload containing bytes to transmit.
 *
 * <h2>Output format</h2>
 *
 * A packet message with a payload containing LoRa modulation symbols.
 * The format of the packet payload is a buffer of unsigned shorts.
 * A 16-bit short can fit all size symbols from 7 to 12 bits.
 *
 * |category /LoRa
 * |keywords lora
 *
 * |param sf[Spread factor] The spreading factor sets the bits per symbol.
 * |default 10
 *
 * |param cr[Coding Rate] The number of error correction bits.
 * |option [4/4] "4/4"
 * |option [4/5] "4/5"
 * |option [4/6] "4/6"
 * |option [4/7] "4/7"
 * |option [4/8] "4/8"
 * |default "4/8"
 *
 * |param whitening Enable/disable whitening of the input message.
 * |option [On] true
 * |option [Off] false
 * |default true
 *
 * |factory /lora/lora_encoder()
 * |param setSpreadFactor(sf)
 * |param setCodingRate(cr)
 * |param enableWhitening(whitening)
 **********************************************************************/
class LoRaEncoder : public Pothos::Block
{
public:
    LoRaEncoder(void):
        _sf(10),
        _rdd(4),
        _whitening(true)
    {
        this->registerCall(this, POTHOS_FCN_TUPLE(LoRaEncoder, setSpreadFactor));
        this->registerCall(this, POTHOS_FCN_TUPLE(LoRaEncoder, setCodingRate));
        this->registerCall(this, POTHOS_FCN_TUPLE(LoRaEncoder, enableWhitening));
        this->setupInput("0");
        this->setupOutput("0");
    }

    static Block *make(void)
    {
        return new LoRaEncoder();
    }

    void setSpreadFactor(const size_t sf)
    {
        _sf = sf;
    }

    void setCodingRate(const std::string &cr)
    {
        if (cr == "4/4") _rdd = 0;
        //else if (cr == "4/5") _rdd = 1;
        //else if (cr == "4/6") _rdd = 2;
        else if (cr == "4/7") _rdd = 3;
        else if (cr == "4/8") _rdd = 4;
        else throw Pothos::InvalidArgumentException("LoRaEncoder::setCodingRate("+cr+")", "unknown coding rate");
    }

    void enableWhitening(const bool whitening)
    {
        _whitening = whitening;
    }

    void work(void)
    {
        auto inPort = this->input(0);
        auto outPort = this->output(0);
        if (not inPort->hasMessage()) return;

        //4 + RDD symbols out for every PPM codewords
        const size_t PPM = _sf; //could be less for reduced set

        //extract the input bytes
        auto msg = inPort->popMessage();
        auto pkt = msg.extract<Pothos::Packet>();
        std::vector<uint8_t> bytes(pkt.payload.length+2);
        std::memcpy(bytes.data()+1, pkt.payload.as<const void *>(), pkt.payload.length);
        const size_t numCodewords = roundUp(bytes.size()*2, PPM);
        const size_t numSymbols = (numCodewords/PPM)*(4 + _rdd);

        //add header and CRC
        bytes[0] = bytes.size();
        bytes[bytes.size()-1] = checksum8(bytes.data(), bytes.size()-1);

        //perform whitening
        if (_whitening)
            SX1232RadioComputeWhitening(bytes.data(), bytes.size());

        //encode each byte as 2 codeword bytes
        std::vector<uint8_t> codewords(numCodewords);
        if (_rdd == 0) for (size_t i = 0; i < bytes.size(); i++)
        {
            codewords[i*2+0] = bytes[i] >> 4;
            codewords[i*2+1] = bytes[i] & 0xf;
        }
        else if (_rdd == 3) for (size_t i = 0; i < bytes.size(); i++)
        {
            codewords[i*2+0] = encodeHamming74(bytes[i] >> 4);
            codewords[i*2+1] = encodeHamming74(bytes[i] & 0xf);
        }
        else if (_rdd == 4) for (size_t i = 0; i < bytes.size(); i++)
        {
            codewords[i*2+0] = encodeHamming84(bytes[i] >> 4);
            codewords[i*2+1] = encodeHamming84(bytes[i] & 0xf);
        }

        //interleave the codewords into symbols
        std::vector<uint16_t> symbols(numSymbols);
        diagonalInterleave(codewords.data(), numCodewords, symbols.data(), PPM, _rdd);

        //gray decode, when SF > PPM, pad out LSBs
        for (auto &sym : symbols)
        {
            sym = grayToBinary16(sym);
            sym <<= (_sf-PPM);
        }

        //post the output symbols
        Pothos::Packet out;
        out.payload = Pothos::BufferChunk(typeid(uint16_t), symbols.size());
        std::memcpy(out.payload.as<void *>(), symbols.data(), out.payload.length);
        outPort->postMessage(out);
    }

private:
    size_t _sf;
    size_t _rdd;
    bool _whitening;
};

static Pothos::BlockRegistry registerLoRaEncoder(
    "/lora/lora_encoder", &LoRaEncoder::make);
