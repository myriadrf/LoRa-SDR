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
 * |default 8
 *
 * |factory /lora/lora_encoder(sf)
 **********************************************************************/
class LoRaEncoder : public Pothos::Block
{
public:
    LoRaEncoder(const size_t sf):
        _sf(sf)
    {
        this->setupInput("0");
        this->setupOutput("0");
    }

    static Block *make(const size_t sf)
    {
        return new LoRaEncoder(sf);
    }

    void work(void)
    {
        auto inPort = this->input(0);
        auto outPort = this->output(0);
        if (not inPort->hasMessage()) return;

        const size_t RDD = 4; //setup for hamming 8, 4 for now
        const size_t PPM = 4 + RDD;

        //extract the input bytes
        auto msg = inPort->popMessage();
        auto pkt = msg.extract<Pothos::Packet>();
        std::vector<uint8_t> bytes(pkt.payload.length+2);
        std::memcpy(bytes.data()+1, pkt.payload.as<const void *>(), pkt.payload.length);
        size_t numCodewords = bytes.size()*2;
        numCodewords = PPM*((numCodewords+(PPM-1))/PPM);

        //add header and CRC
        bytes[0] = bytes.size();
        bytes[bytes.size()-1] = checksum8(bytes.data(), bytes.size()-1);

        //perform whitening
        SX1232RadioComputeWhitening(bytes.data(), bytes.size());

        //encode each byte as 2 codeword bytes
        std::vector<uint8_t> codewords(numCodewords);
        for (size_t i = 0; i < bytes.size(); i++)
        {
            codewords[i*2+0] = encodeHamming84(bytes[i] >> 4);
            codewords[i*2+1] = encodeHamming84(bytes[i] & 0xf);
        }

        //interleave the codewords into symbols
        std::vector<uint16_t> symbols(codewords.size());
        /*
        for (size_t off = 0; off < symbols.size(); off+=PPM)
        {
            for (size_t k = 0; k < 4 + RDD; k++)
            {
                for (size_t m = 0; m < PPM; m++)
                {
                    const size_t i = (m-k)%PPM;
                    const auto bit = (codewords[off + i] >> k) & 0x1;
                    symbols[off + k] |= (bit << m);
                }
            }
        }
        */
        for (size_t i = 0; i < codewords.size(); i++) symbols[i] = codewords[i];

        //gray decode, when SF > PPM, pad out LSBs
        for (auto &sym : symbols)
        {
            sym = grayToBinary16(sym);
            if (_sf > PPM) sym <<= (_sf-PPM);
        }

        //post the output symbols
        Pothos::Packet out;
        out.payload = Pothos::BufferChunk(typeid(uint16_t), symbols.size());
        std::memcpy(out.payload.as<void *>(), symbols.data(), out.payload.length);
        outPort->postMessage(out);
    }

private:
    const size_t _sf;
};

static Pothos::BlockRegistry registerLoRaEncoder(
    "/lora/lora_encoder", &LoRaEncoder::make);
