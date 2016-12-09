// Copyright (c) 2016-2016 Lime Microsystems
// Copyright (c) 2016-2016 Arne Hennig
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
 * |param hdr[Header Output] Enable/disable header output.
 * |option [On] true
 * |option [Off] false
 * |default false
 *
 * |param dataLength implicit data length.
 * |default 8
 *
 * |param crcc Enable/disable crc check of the decoded message.
 * |option [On] true
 * |option [Off] false
 * |default false
 *
 * |param whitening Enable/disable whitening of the decoded message.
 * |option [On] true
 * |option [Off] false
 * |default true
 *
 * |param interleaving Enable/disable interleaving of the decoded message.
 * |option [On] true
 * |option [Off] false
 * |default true
 *
 * |param errorCheck Enable/disable error checking.
 * |option [On] true
 * |option [Off] false
 * |default true
 *
 * |factory /lora/lora_decoder()
 * |setter setSpreadFactor(sf)
 * |setter setSymbolSize(ppm)
 * |setter setCodingRate(cr)
 * |setter enableExplicit(explicit)
 * |setter enableHdr(hdr)
 * |setter setDataLength(dataLength)
 * |setter enableCrcc(crcc)
 * |setter enableWhitening(whitening)
 * |setter enableInterleaving(interleaving)
 * |setter enableErrorCheck(errorCheck)
 **********************************************************************/
class LoRaDecoder : public Pothos::Block
{
public:
    LoRaDecoder(void):
        _sf(10),
        _ppm(0),
        _rdd(4),
        _whitening(true),
		_crcc(false),
		_interleaving(true),
		_errorCheck(false),
		_explicit(true),
        _hdr(false),
		_dataLength(8),
        _dropped(0)
    {
        this->registerCall(this, POTHOS_FCN_TUPLE(LoRaDecoder, setSpreadFactor));
        this->registerCall(this, POTHOS_FCN_TUPLE(LoRaDecoder, setSymbolSize));
        this->registerCall(this, POTHOS_FCN_TUPLE(LoRaDecoder, setCodingRate));
        this->registerCall(this, POTHOS_FCN_TUPLE(LoRaDecoder, enableWhitening));
		this->registerCall(this, POTHOS_FCN_TUPLE(LoRaDecoder, enableCrcc));
		this->registerCall(this, POTHOS_FCN_TUPLE(LoRaDecoder, enableInterleaving));
		this->registerCall(this, POTHOS_FCN_TUPLE(LoRaDecoder, enableExplicit));
        this->registerCall(this, POTHOS_FCN_TUPLE(LoRaDecoder, enableHdr));
		this->registerCall(this, POTHOS_FCN_TUPLE(LoRaDecoder, setDataLength));
		this->registerCall(this, POTHOS_FCN_TUPLE(LoRaDecoder, enableErrorCheck));
        this->registerCall(this, POTHOS_FCN_TUPLE(LoRaDecoder, getDropped));

        this->registerSignal("dropped");
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
        else if (cr == "4/5") _rdd = 1;
        else if (cr == "4/6") _rdd = 2;
        else if (cr == "4/7") _rdd = 3;
        else if (cr == "4/8") _rdd = 4;
        else throw Pothos::InvalidArgumentException("LoRaDecoder::setCodingRate("+cr+")", "unknown coding rate");
    }

    void enableWhitening(const bool whitening)
    {
        _whitening = whitening;
    }

	void enableInterleaving(const bool interleaving)
	{
		_interleaving = interleaving;
	}

	void enableExplicit(const bool __explicit) {
		_explicit = __explicit;
	}
    
    void enableHdr(const bool hdr) {
        _hdr = hdr;
    }

	void enableErrorCheck(const bool errorCheck) {
		_errorCheck = errorCheck;
	}

	void enableCrcc(const bool crcc)
	{
		_crcc = crcc;
	}

	void setDataLength(const size_t dataLength)
	{
		_dataLength = dataLength;
	}

    unsigned long long getDropped(void) const
    {
        return _dropped;
    }

    void activate(void)
    {
        _dropped = 0;
        this->emitSignal("dropped", _dropped);
    }

	void work(void){
		auto inPort = this->input(0);
		auto outPort = this->output(0);
		if (not inPort->hasMessage()) return;

		const size_t PPM = (_ppm == 0) ? _sf : _ppm;
		if (PPM > _sf) throw Pothos::Exception("LoRaDecoder::work()", "failed check: PPM <= SF");

		//extract the input symbols
		auto msg = inPort->popMessage();
		auto pkt = msg.extract<Pothos::Packet>();
        
        if (pkt.payload.elements() < N_HEADER_SYMBOLS) return; // need at least a header
        
		const size_t numSymbols = roundUp(pkt.payload.elements(), 4 + _rdd);
		const size_t numCodewords = (numSymbols / (4 + _rdd))*PPM;
		std::vector<uint16_t> symbols(numSymbols);
		std::memcpy(symbols.data(), pkt.payload.as<const void *>(), pkt.payload.length);

        int rdd = _rdd; //make a copy to be changed in header decode

		//gray encode, when SF > PPM, depad the LSBs with rounding
		for (auto &sym : symbols){
			sym += (1 << (_sf - PPM)) / 2; //increment by 1/2
			sym >>= (_sf - PPM); //down shift to PPM bits
			sym = binaryToGray16(sym);
		}
		//deinterleave / dewhiten the symbols into codewords
		std::vector<uint8_t> codewords(numCodewords);
		if (_interleaving) {
			size_t sOfs = 0;
			size_t cOfs = 0;
			if (rdd != HEADER_RDD) {
				diagonalDeterleaveSx(symbols.data(), N_HEADER_SYMBOLS, codewords.data(), PPM, HEADER_RDD);
				if (_explicit) {
					Sx1272ComputeWhiteningLfsr(codewords.data() + N_HEADER_CODEWORDS, PPM - N_HEADER_CODEWORDS, 0, HEADER_RDD);
				}
				else {
					Sx1272ComputeWhiteningLfsr(codewords.data(), PPM, 0, HEADER_RDD);
				}
				cOfs += PPM;
				sOfs += N_HEADER_SYMBOLS;
				if (numSymbols - sOfs > 0) {
					diagonalDeterleaveSx(symbols.data() + sOfs, numSymbols-sOfs, codewords.data() + cOfs, PPM, rdd);
					if (_explicit) {
						Sx1272ComputeWhiteningLfsr(codewords.data() + cOfs, numCodewords - cOfs, PPM-N_HEADER_CODEWORDS, rdd);
					}
					else {
						Sx1272ComputeWhiteningLfsr(codewords.data() + cOfs, numCodewords - cOfs, PPM, rdd);
					}
				}
			}else{
				diagonalDeterleaveSx(symbols.data(), numSymbols, codewords.data(), PPM, rdd);
				if (_explicit) {
					Sx1272ComputeWhiteningLfsr(codewords.data()+N_HEADER_CODEWORDS, numCodewords-N_HEADER_CODEWORDS, 0, rdd);
				}
				else {
					Sx1272ComputeWhiteningLfsr(codewords.data(), numCodewords, 0, rdd);
				}
			}
		/*
			Pothos::Packet out;
			out.payload = Pothos::BufferChunk(typeid(uint8_t), numCodewords);
			std::memcpy(out.payload.as<void *>(), codewords.data(), numCodewords);
			outPort->postMessage(out);
			return;
		*/
		}
		else {
			Pothos::Packet out;
			out.payload = Pothos::BufferChunk(typeid(uint16_t), numSymbols);
			std::memcpy(out.payload.as<void *>(), symbols.data(), out.payload.length);
			outPort->postMessage(out);
			return;
		}

		bool error = false;
		bool bad = false;
		std::vector<uint8_t> bytes((codewords.size()+1) / 2);
		size_t dOfs = 0;
		size_t cOfs = 0;
        
        size_t packetLength = 0;
        size_t dataLength = 0;
        bool checkCrc = _crcc;
		
		if (_explicit) {
			bytes[0] = decodeHamming84sx(codewords[1], error, bad) & 0xf;
			bytes[0] |= decodeHamming84sx(codewords[0], error, bad) << 4;	// length

			bytes[1] = decodeHamming84sx(codewords[2], error, bad) & 0xf;	// coding rate and crc enable

			bytes[2] = decodeHamming84sx(codewords[4], error, bad) & 0xf;
			bytes[2] |= decodeHamming84sx(codewords[3], error, bad) << 4;	// checksum
			
			bytes[2] ^= headerChecksum(bytes.data());

			if (error && _errorCheck) return this->drop();
            
            if (0 == (bytes[1] & 1)) checkCrc = false;	// disable crc check if not present in the packet
            rdd = (bytes[1] >> 1) & 0x7;				// header contains error correction info
            if (rdd > 4) return this->drop();
            
            packetLength = bytes[0];
            dataLength = packetLength + ((bytes[1] & 1)?5:3);  // include  header and crc
            
			cOfs = N_HEADER_CODEWORDS;
			dOfs = 6;
        }else{
            packetLength = _dataLength;
            if (_crcc){
                dataLength = packetLength + 2;
            }else{
                dataLength = packetLength;
            }
        }
        
        if (dataLength > bytes.size()) return this->drop();
		
		for (; cOfs < PPM; cOfs++, dOfs++) {
			if (dOfs & 1)
				bytes[dOfs >> 1] |= decodeHamming84sx(codewords[cOfs], error, bad) << 4;
			else
				bytes[dOfs >> 1] = decodeHamming84sx(codewords[cOfs], error, bad) & 0xf;
		}

		if (dOfs & 1) {
			if (rdd == 0){
				bytes[dOfs>>1] |= codewords[cOfs++] << 4;
			}
			else if (rdd == 1){
				bytes[dOfs >> 1] |= checkParity54(codewords[cOfs++], error) << 4;
			}
			else if (rdd == 2) {
				bytes[dOfs >> 1] |= checkParity64(codewords[cOfs++], error) << 4;
			}
			else if (rdd == 3){
				bytes[dOfs >> 1] |= decodeHamming74sx(codewords[cOfs++], error) << 4;
			}
			else if (rdd == 4){
				bytes[dOfs >> 1] |= decodeHamming84sx(codewords[cOfs++], error, bad) << 4;
			}
			dOfs++;
		}
		dOfs >>= 1;

		if (error && _errorCheck) return this->drop();


		//decode each codeword as 2 bytes with correction
		if (rdd == 0) for (size_t i = dOfs; i < dataLength; i++) {
			bytes[i] = codewords[cOfs++] & 0xf;
			bytes[i] |= codewords[cOfs++] << 4;
		}else if (rdd == 1) for (size_t i = dOfs; i < dataLength; i++) {
			bytes[i] = checkParity54(codewords[cOfs++],error);
			bytes[i] |= checkParity54(codewords[cOfs++], error) << 4;
		}else if (rdd == 2) for (size_t i = dOfs; i < dataLength; i++) {
			bytes[i] = checkParity64(codewords[cOfs++], error);
			bytes[i] |= checkParity64(codewords[cOfs++],error) << 4;
		}else if (rdd == 3) for (size_t i = dOfs; i < dataLength; i++){
			bytes[i] = decodeHamming74sx(codewords[cOfs++], error) & 0xf;
			bytes[i] |= decodeHamming74sx(codewords[cOfs++], error) << 4;
		}else if (rdd == 4) for (size_t i = dOfs; i < dataLength; i++){
			bytes[i] = decodeHamming84sx(codewords[cOfs++], error, bad) & 0xf;
			bytes[i] |= decodeHamming84sx(codewords[cOfs++], error, bad) << 4;
		}
		
		if (error && _errorCheck) return this->drop();
        
        dOfs = 0;
        
		if (_explicit) {
			if (bytes[1] & 1) {							// always compute crc if present
				uint16_t crc = sx1272DataChecksum(bytes.data() + 3, packetLength);
				uint16_t packetCrc = bytes[3 + packetLength] | (bytes[4 + packetLength] << 8);
				if (crc != packetCrc && checkCrc) return this->drop();
				bytes[3 + packetLength] ^= crc;
				bytes[4 + packetLength] ^= (crc >> 8);
			}
            if (!_hdr){
                dOfs = 3;
                dataLength -= 5;
            }
		}
		else {
            if (checkCrc) {
                uint16_t crc = sx1272DataChecksum(bytes.data(), _dataLength);
                uint16_t packetCrc = bytes[_dataLength] | (bytes[_dataLength + 1] << 8);
                if (crc != packetCrc) return this->drop();
                bytes[_dataLength + 0] ^= crc;
                bytes[_dataLength + 1] ^= (crc >> 8);
            }
		}
		
		//post the output bytes
		Pothos::Packet out;
		out.payload = Pothos::BufferChunk(typeid(uint8_t), dataLength);
		std::memcpy(out.payload.as<void *>(), bytes.data()+dOfs, out.payload.length);
		outPort->postMessage(out);
		return;

    }

private:

    void drop(void)
    {
        _dropped++;
        this->emitSignal("dropped", _dropped);
    }

    size_t _sf;
    size_t _ppm;
    size_t _rdd;
    bool _whitening;
	bool _crcc;
	bool _interleaving;
	bool _errorCheck;
	bool _explicit;
    bool _hdr;
	size_t _dataLength;
    unsigned long long _dropped;
};

static Pothos::BlockRegistry registerLoRaDecoder(
    "/lora/lora_decoder", &LoRaDecoder::make);
