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
 * |default 8
 *
 * |factory /lora/lora_decoder(sf)
 **********************************************************************/
class LoRaDecoder : public Pothos::Block
{
public:
    LoRaDecoder(const size_t sf):
        _sf(sf)
    {
        this->setupInput("0");
        this->setupOutput("0");
    }

    static Block *make(const size_t sf)
    {
        return new LoRaDecoder(sf);
    }

    void work(void)
    {
        auto inPort = this->input(0);
        auto outPort = this->output(0);
        if (not inPort->hasMessage()) return;

        const size_t RDD = 4; //setup for hamming 8, 4 for now
        const size_t PPM = 4 + RDD;

        //extract the input symbols
        auto msg = inPort->popMessage();
        auto pkt = msg.extract<Pothos::Packet>();
        size_t numSymbols = pkt.payload.elements();
        numSymbols = PPM*((numSymbols+(PPM-1))/PPM);
        std::vector<uint16_t> symbols(numSymbols);
        std::memcpy(symbols.data(), pkt.payload.as<const void *>(), pkt.payload.length);

        //gray encode, when SF > PPM, depad the LSBs
        for (auto &sym : symbols)
        {
            sym = binaryToGray16(sym);
            if (_sf > PPM) sym >>= (_sf-PPM);
        }

        //deinterleave the symbols into codewords
        std::vector<uint16_t> codewords(symbols.size());
        for (size_t off = 0; off < symbols.size(); off+=PPM)
        {
            for (size_t k = 0; k < 4 + RDD; k++)
            {
                for (size_t m = 0; m < PPM; m++)
                {
                    const size_t i = (m-k)%PPM;
                    const auto bit = (symbols[off + k] >> m) & 0x1;
                    codewords[off + i] |= (bit << k);
                }
            }
        }
        //for (size_t i = 0; i < codewords.size(); i++) codewords[i] = symbols[i];

        //decode each codeword as 2 bytes with correction
        bool error = false;
        std::vector<uint8_t> bytes(codewords.size()/2);
        for (size_t i = 0; i < bytes.size(); i++)
        {
            bytes[i] = decodeHamming84(codewords[i*2+0], error) << 4;
            bytes[i] |= decodeHamming84(codewords[i*2+1], error) & 0xf;
        }
        //if (error) return;

        //undo the whitening
        SX1232RadioComputeWhitening(bytes.data(), bytes.size());

        //header and crc
        size_t length = bytes[0];
        if (length < 2) return;
        if (length > bytes.size()) return;
        uint8_t crc = checksum8(bytes.data(), length-1);
        if (crc != bytes[length-1]) return;

        //post the output bytes
        Pothos::Packet out;
        out.payload = Pothos::BufferChunk(typeid(uint8_t), length-2);
        std::memcpy(out.payload.as<void *>(), bytes.data()+1, out.payload.length);
        outPort->postMessage(out);
    }

private:
    const size_t _sf;
};

static Pothos::BlockRegistry registerLoRaDecoder(
    "/lora/lora_decoder", &LoRaDecoder::make);
