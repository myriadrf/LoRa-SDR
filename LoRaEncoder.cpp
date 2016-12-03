// Copyright (c) 2016-2016 Lime Microsystems
// Copyright (c) 2016-2016 Arne Hennig
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
 * |param ppm[Symbol size] The size of the symbol set (_ppm &lt;= SF).
 * Specify _ppm less than the spread factor to use a reduced symbol set.
 * The special value of zero uses the full symbol set (PPM == SF).
 * |default 0
 * |option [Full set] 0
 * |widget ComboBox(editable=true)
 * |preview valid
 *
 * |param cr[Coding Rate] The number of error correction bits.
 * |option [4/4] "4/4"
 * |option [4/5] "4/5"
 * |option [4/6] "4/6"
 * |option [4/7] "4/7"
 * |option [4/8] "4/8"
 * |default "4/8"
 *
 * |param explicit Enable/disable explicit header mode.
 * |option [On] true
 * |option [Off] false
 * |default true
 *
 * |param crc Enable/disable crc.
 * |option [On] true
 * |option [Off] false
 * |default true
 *
 * |param whitening Enable/disable whitening of the input message.
 * |option [On] true
 * |option [Off] false
 * |default true
 *
 * |factory /lora/lora_encoder()
 * |setter setSpreadFactor(sf)
 * |setter setSymbolSize(ppm)
 * |setter setCodingRate(cr)
 * |setter enableExplicit(explicit)
 * |setter enableCrc(crc)
 * |setter enableWhitening(whitening)
 **********************************************************************/
class LoRaEncoder : public Pothos::Block
{
public:
	LoRaEncoder(void) :
		_sf(10),
		_ppm(0),
		_rdd(4),
		_explicit(true),
		_crc(true),
		_whitening(true)
	{
		this->registerCall(this, POTHOS_FCN_TUPLE(LoRaEncoder, setSpreadFactor));
		this->registerCall(this, POTHOS_FCN_TUPLE(LoRaEncoder, setSymbolSize));
		this->registerCall(this, POTHOS_FCN_TUPLE(LoRaEncoder, setCodingRate));
		this->registerCall(this, POTHOS_FCN_TUPLE(LoRaEncoder, enableWhitening));
		this->registerCall(this, POTHOS_FCN_TUPLE(LoRaEncoder, enableExplicit));
		this->registerCall(this, POTHOS_FCN_TUPLE(LoRaEncoder, enableCrc));
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

	void setSymbolSize(const size_t ppm)
	{
		_ppm = ppm;
	}

	void setCodingRate(const std::string &cr)
	{
		if (cr == "4/4") _rdd = 0;
		else if (cr == "4/5") _rdd = 1;
		else if (cr == "4/6") _rdd = 2;
		else if (cr == "4/7") _rdd = 3;
		else if (cr == "4/8") _rdd = 4;
		else throw Pothos::InvalidArgumentException("LoRaEncoder::setCodingRate(" + cr + ")", "unknown coding rate");
	}

	void enableWhitening(const bool whitening)
	{
		_whitening = whitening;
	}

	void enableExplicit(const bool __explicit) {
		_explicit = __explicit;
	}

	void enableCrc(const bool crc) {
		_crc = crc;
	}

	void encodeFec(std::vector<uint8_t> &codewords, const size_t RDD, size_t &cOfs, size_t &dOfs, const uint8_t *bytes, const size_t count) {
		if (RDD == 0) for (size_t i = 0; i < count; i++, dOfs++) {
			if (dOfs & 1)
				codewords[cOfs++] = bytes[dOfs >> 1] >> 4;
			else
				codewords[cOfs++] = bytes[dOfs >> 1] & 0xf;
		} else if (RDD == 1) for (size_t i = 0; i < count; i++, dOfs++) {
			if (dOfs & 1)
				codewords[cOfs++] = encodeParity54(bytes[dOfs >> 1] >> 4);
			else
				codewords[cOfs++] = encodeParity54(bytes[dOfs >> 1] & 0xf);
		} else if (RDD == 2) for (size_t i = 0; i < count; i++, dOfs++) {
			if (dOfs & 1)
				codewords[cOfs++] = encodeParity64(bytes[dOfs >> 1] >> 4);
			else
				codewords[cOfs++] = encodeParity64(bytes[dOfs >> 1] & 0xf);
		} else if (RDD == 3) for (size_t i = 0; i < count; i++, dOfs++) {
			if (dOfs & 1)
				codewords[cOfs++] = encodeHamming74sx(bytes[dOfs >> 1] >> 4);
			else
				codewords[cOfs++] = encodeHamming74sx(bytes[dOfs >> 1] & 0xf);
		} else if (RDD == 4) for (size_t i = 0; i < count; i++, dOfs++) {
			if (dOfs & 1)
				codewords[cOfs++] = encodeHamming84sx(bytes[dOfs >> 1] >> 4);
			else
				codewords[cOfs++] = encodeHamming84sx(bytes[dOfs >> 1] & 0xf);
		}
	}

	void work(void) {
		auto inPort = this->input(0);
		auto outPort = this->output(0);
		if (not inPort->hasMessage()) return;
		const size_t PPM = (_ppm == 0) ? _sf : _ppm;
		if (PPM > _sf) throw Pothos::Exception("LoRaEncoder::work()", "failed check: PPM <= SF");

		//extract the input bytes
		auto msg = inPort->popMessage();
		auto pkt = msg.extract<Pothos::Packet>();
		size_t payloadLength = pkt.payload.length + (_crc ? 2 : 0);
		std::vector<uint8_t> bytes(payloadLength);
		std::memcpy(bytes.data(), pkt.payload.as<const void *>(), pkt.payload.length);
				
		const size_t numCodewords = roundUp(bytes.size() * 2 + (_explicit ? N_HEADER_CODEWORDS:0), PPM);
		const size_t numSymbols = N_HEADER_SYMBOLS + (numCodewords / PPM - 1) * (4 + _rdd);		// header is always coded with 8 bits
		
		size_t cOfs = 0;
		size_t dOfs = 0;
		std::vector<uint8_t> codewords(numCodewords);

		if (_crc) {
			uint16_t crc = sx1272DataChecksum(bytes.data(),  pkt.payload.length);
			bytes[pkt.payload.length] = crc & 0xff;
			bytes[pkt.payload.length+1] = (crc >> 8) & 0xff;
		}

		if (_explicit) {
			std::vector<uint8_t> hdr(3);
			uint8_t len = pkt.payload.length;
			hdr[0] = len;
			hdr[1] = (_crc ? 1 : 0) | (_rdd << 1);
			hdr[2] = headerChecksum(hdr.data());

			codewords[cOfs++] = encodeHamming84sx(hdr[0] >> 4);
			codewords[cOfs++] = encodeHamming84sx(hdr[0] & 0xf);	// length
			codewords[cOfs++] = encodeHamming84sx(hdr[1] & 0xf);	// crc / fec info
			codewords[cOfs++] = encodeHamming84sx(hdr[2] >> 4);		// checksum
			codewords[cOfs++] = encodeHamming84sx(hdr[2] & 0xf);
		}
		size_t cOfs1 = cOfs;
		encodeFec(codewords, 4, cOfs, dOfs, bytes.data(), PPM - cOfs);
		if (_whitening) {
			Sx1272ComputeWhitening(codewords.data() + cOfs1, PPM - cOfs1, 0, HEADER_RDD);
		}

		if (numCodewords > PPM) {
			size_t cOfs2 = cOfs;
			encodeFec(codewords, _rdd, cOfs, dOfs, bytes.data(), numCodewords-PPM);
			if (_whitening) {
				Sx1272ComputeWhitening(codewords.data() + cOfs2, numCodewords - PPM, PPM - cOfs1, _rdd);
			}
		}

		//interleave the codewords into symbols
		std::vector<uint16_t> symbols(numSymbols);
		diagonalInterleaveSx(codewords.data(), PPM, symbols.data(), PPM, HEADER_RDD);
		if (numCodewords > PPM) {
			diagonalInterleaveSx(codewords.data() + PPM, numCodewords-PPM, symbols.data()+N_HEADER_SYMBOLS, PPM, _rdd);
		}

		//gray decode, when SF > PPM, pad out LSBs
		for (auto &sym : symbols){
			sym = grayToBinary16(sym);
			sym <<= (_sf - PPM);
		}

		//post the output symbols
		Pothos::Packet out;
		out.payload = Pothos::BufferChunk(typeid(uint16_t), symbols.size());
		std::memcpy(out.payload.as<void *>(), symbols.data(), out.payload.length);
		outPort->postMessage(out);
	}

private:
    size_t _sf;
    size_t _ppm;
	size_t _rdd;
	bool _explicit;
	bool _crc;
    bool _whitening;
};

static Pothos::BlockRegistry registerLoRaEncoder(
    "/lora/lora_encoder", &LoRaEncoder::make);
