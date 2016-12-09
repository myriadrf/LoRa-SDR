/***********************************************************************
 * Defines
 **********************************************************************/
#define HEADER_RDD          4
#define N_HEADER_SYMBOLS    (HEADER_RDD + 4)
#define N_HEADER_CODEWORDS  5


/***********************************************************************
 * Round functions
 **********************************************************************/
static inline unsigned roundUp(unsigned num, unsigned factor)
{
    return ((num + factor - 1) / factor) * factor;
}

/***********************************************************************
 * Simple 8-bit checksum routine
 **********************************************************************/
static inline uint8_t checksum8(const uint8_t *p, const size_t len)
{
    uint8_t acc = 0;
    for (size_t i = 0; i < len; i++)
    {
        acc = (acc >> 1) + ((acc & 0x1) << 7); //rotate
        acc += p[i]; //add
    }
    return acc;
}

static inline uint8_t headerChecksum(const uint8_t *h) {
	auto a0 = (h[0] >> 4) & 0x1;
	auto a1 = (h[0] >> 5) & 0x1;
	auto a2 = (h[0] >> 6) & 0x1;
	auto a3 = (h[0] >> 7) & 0x1;

	auto b0 = (h[0] >> 0) & 0x1;
	auto b1 = (h[0] >> 1) & 0x1;
	auto b2 = (h[0] >> 2) & 0x1;
	auto b3 = (h[0] >> 3) & 0x1;

	auto c0 = (h[1] >> 0) & 0x1;
	auto c1 = (h[1] >> 1) & 0x1;
	auto c2 = (h[1] >> 2) & 0x1;
	auto c3 = (h[1] >> 3) & 0x1;

	uint8_t res;
	res = (a0 ^ a1 ^ a2 ^ a3) << 4;
	res |= (a3 ^ b1 ^ b2 ^ b3 ^ c0) << 3;
	res |= (a2 ^ b0 ^ b3 ^ c1 ^ c3) << 2;
	res |= (a1 ^ b0 ^ b2 ^ c0 ^ c1 ^ c2) << 1;
	res |= a0 ^ b1 ^ c0 ^ c1 ^ c2 ^ c3;
	
	return res;
}

static inline uint16_t crc16sx(uint16_t crc, const uint16_t poly) {
	for (int i = 0; i < 8; i++) {
		if (crc & 0x8000) {
			crc = (crc << 1) ^ poly;
		}
		else {
			crc <<= 1;
		}
	}
	return crc;
}

static inline uint8_t xsum8(uint8_t t) {
	t ^= t >> 4;
	t ^= t >> 2;
	t ^= t >> 1;
	return (t & 1);
}

/***********************************************************************
 *  CRC reverse engineered from Sx1272 data stream.
 *  Modified CCITT crc with masking of the output with an 8bit lfsr
 **********************************************************************/
static inline uint16_t sx1272DataChecksum(const uint8_t *data, int length) {
	uint16_t res = 0;
	uint8_t v = 0xff;
	uint16_t crc = 0;
	for (int i = 0; i < length; i++) {
		crc = crc16sx(res, 0x1021);
		v = xsum8(v & 0xB8) | (v << 1);
		res = crc ^ data[i];
	}
	res ^= v; 
	v = xsum8(v & 0xB8) | (v << 1);
	res ^= v << 8;
	return res;
}


/***********************************************************************
 *  http://www.semtech.com/images/datasheet/AN1200.18_AG.pdf
 **********************************************************************/
static inline void SX1232RadioComputeWhitening( uint8_t *buffer, uint16_t bufferSize )
{
    uint8_t WhiteningKeyMSB; // Global variable so the value is kept after starting the
    uint8_t WhiteningKeyLSB; // de-whitening process
    WhiteningKeyMSB = 0x01; // Init value for the LFSR, these values should be initialize only
    WhiteningKeyLSB = 0xFF; // at the start of a whitening or a de-whitening process
    // *buffer is a char pointer indicating the data to be whiten / de-whiten
    // buffersize is the number of char to be whiten / de-whiten
    // >> The whitened / de-whitened data are directly placed into the pointer
    uint8_t i = 0;
    uint16_t j = 0;
    uint8_t WhiteningKeyMSBPrevious = 0; // 9th bit of the LFSR
    for( j = 0; j < bufferSize; j++ )     // byte counter
    {
        buffer[j] ^= WhiteningKeyLSB;   // XOR between the data and the whitening key
        for( i = 0; i < 8; i++ )    // 8-bit shift between each byte
        {
            WhiteningKeyMSBPrevious = WhiteningKeyMSB;
            WhiteningKeyMSB = ( WhiteningKeyLSB & 0x01 ) ^ ( ( WhiteningKeyLSB >> 5 ) & 0x01 );
            WhiteningKeyLSB= ( ( WhiteningKeyLSB >> 1 ) & 0xFF ) | ( ( WhiteningKeyMSBPrevious << 7 ) & 0x80 );
        }
    }
}


/***********************************************************************
 *  Whitening generator reverse engineered from Sx1272 data stream.
 *  Each bit of a codeword is combined with the output from a different position in the whitening sequence.
 **********************************************************************/
static inline void Sx1272ComputeWhitening(uint8_t *buffer, uint16_t bufferSize, const int bitOfs, const int RDD) {
	static const int ofs0[8] = {6,4,2,0,-112,-114,-302,-34 };	// offset into sequence for each bit
	static const int ofs1[5] = {6,4,2,0,-360 };					// different offsets used for single parity mode (1 == RDD)
	static const int whiten_len = 510;							// length of whitening sequence
	static const uint64_t whiten_seq[8] = {						// whitening sequence
		0x0102291EA751AAFFL,0xD24B050A8D643A17L,0x5B279B671120B8F4L,0x032B37B9F6FB55A2L,
		0x994E0F87E95E2D16L,0x7CBCFC7631984C26L,0x281C8E4F0DAEF7F9L,0x1741886EB7733B15L
	};
	const int *ofs = (1 == RDD) ? ofs1 : ofs0;
	int i, j;
	for (j = 0; j < bufferSize; j++) {
		uint8_t x = 0;
		for (i = 0; i < 4 + RDD; i++) {
			int t = (ofs[i] + j + bitOfs + whiten_len) % whiten_len;
			if (whiten_seq[t >> 6] & ((uint64_t)1 << (t & 0x3F))) {
				x |= 1 << i;
			}
		}
		buffer[j] ^= x;
	}	
}

/***********************************************************************
 *  Whitening generator reverse engineered from Sx1272 data stream.
 *  Same as above but using the actual interleaved LFSRs.
 **********************************************************************/
static inline void Sx1272ComputeWhiteningLfsr(uint8_t *buffer, uint16_t bufferSize, const int bitOfs, const size_t RDD) {
    static const uint64_t seed1[2] = {0x6572D100E85C2EFF,0xE85C2EFFFFFFFFFF};   // lfsr start values
    static const uint64_t seed2[2] = {0x05121100F8ECFEEF,0xF8ECFEEFEFEFEFEF};   // lfsr start values for single parity mode (1 == RDD)
    const uint8_t m = 0xff >> (4 - RDD);
    uint64_t r[2] = {(1 == RDD)?seed2[0]:seed1[0],(1 == RDD)?seed2[1]:seed1[1]};
    int i,j;
    for (i = 0; i < bitOfs;i++){
        r[i & 1] = (r[i & 1] >> 8) | (((r[i & 1] >> 32) ^ (r[i & 1] >> 24) ^ (r[i & 1] >> 16) ^ r[i & 1]) << 56);   // poly: 0x1D
    }
    for (j = 0; j < bufferSize; j++,i++) {
        buffer[j] ^= r[i & 1] & m;
        r[i & 1] = (r[i & 1] >> 8) | (((r[i & 1] >> 32) ^ (r[i & 1] >> 24) ^ (r[i & 1] >> 16) ^ r[i & 1]) << 56);
    }	
}

/***********************************************************************
 *  https://en.wikipedia.org/wiki/Gray_code
 **********************************************************************/

/*
 * This function converts an unsigned binary
 * number to reflected binary Gray code.
 *
 * The operator >> is shift right. The operator ^ is exclusive or.
 */
static inline unsigned short binaryToGray16(unsigned short num)
{
    return num ^ (num >> 1);
}

/*
 * A more efficient version, for Gray codes of 16 or fewer bits.
 */
static inline unsigned short grayToBinary16(unsigned short num)
{
    num = num ^ (num >> 8);
    num = num ^ (num >> 4);
    num = num ^ (num >> 2);
    num = num ^ (num >> 1);
    return num;
}

/***********************************************************************
 * Encode a 4 bit word into a 8 bits with parity
 * Non standard version used in sx1272.
 * https://en.wikipedia.org/wiki/Hamming_code
 **********************************************************************/
static inline unsigned char encodeHamming84sx(const unsigned char x)
{
    auto d0 = (x >> 0) & 0x1;
    auto d1 = (x >> 1) & 0x1;
    auto d2 = (x >> 2) & 0x1;
    auto d3 = (x >> 3) & 0x1;
    
    unsigned char b = x & 0xf;
    b |= (d0 ^ d1 ^ d2) << 4;
    b |= (d1 ^ d2 ^ d3) << 5;
    b |= (d0 ^ d1 ^ d3) << 6;
    b |= (d0 ^ d2 ^ d3) << 7;
    return b;
}

/***********************************************************************
 * Decode 8 bits into a 4 bit word with single bit correction.
 * Non standard version used in sx1272.
 * Set error to true when a parity error was detected
 * Set bad to true when the result could not be corrected
 **********************************************************************/
static inline unsigned char decodeHamming84sx(const unsigned char b, bool &error, bool &bad)
{
    auto b0 = (b >> 0) & 0x1;
    auto b1 = (b >> 1) & 0x1;
    auto b2 = (b >> 2) & 0x1;
    auto b3 = (b >> 3) & 0x1;
    auto b4 = (b >> 4) & 0x1;
    auto b5 = (b >> 5) & 0x1;
    auto b6 = (b >> 6) & 0x1;
    auto b7 = (b >> 7) & 0x1;
    
    auto p0 = (b0 ^ b1 ^ b2 ^ b4);
    auto p1 = (b1 ^ b2 ^ b3 ^ b5);
    auto p2 = (b0 ^ b1 ^ b3 ^ b6);
    auto p3 = (b0 ^ b2 ^ b3 ^ b7);
    
    auto parity = (p0 << 0) | (p1 << 1) | (p2 << 2) | (p3 << 3);
    if (parity != 0) error = true;
    switch (parity & 0xf)
    {
        case 0xD: return (b ^ 1) & 0xf;
        case 0x7: return (b ^ 2) & 0xf;
        case 0xB: return (b ^ 4) & 0xf;
        case 0xE: return (b ^ 8) & 0xf;
        case 0x0:
        case 0x1:
        case 0x2:
        case 0x4:
        case 0x8: return b & 0xf;
        default: bad = true; return b & 0xf;
    }
}

/***********************************************************************
 * Encode a 4 bit word into a 7 bits with parity.
 * Non standard version used in sx1272.
 **********************************************************************/
static inline unsigned char encodeHamming74sx(const unsigned char x)
{
    auto d0 = (x >> 0) & 0x1;
    auto d1 = (x >> 1) & 0x1;
    auto d2 = (x >> 2) & 0x1;
    auto d3 = (x >> 3) & 0x1;
    
    unsigned char b = x & 0xf;
    b |= (d0 ^ d1 ^ d2) << 4;
    b |= (d1 ^ d2 ^ d3) << 5;
    b |= (d0 ^ d1 ^ d3) << 6;
    return b;
}

/***********************************************************************
 * Decode 7 bits into a 4 bit word with single bit correction.
 * Non standard version used in sx1272.
 * Set error to true when a parity error was detected
 **********************************************************************/
static inline unsigned char decodeHamming74sx(const unsigned char b, bool &error)
{
    auto b0 = (b >> 0) & 0x1;
    auto b1 = (b >> 1) & 0x1;
    auto b2 = (b >> 2) & 0x1;
    auto b3 = (b >> 3) & 0x1;
    auto b4 = (b >> 4) & 0x1;
    auto b5 = (b >> 5) & 0x1;
    auto b6 = (b >> 6) & 0x1;
    
    auto p0 = (b0 ^ b1 ^ b2 ^ b4);
    auto p1 = (b1 ^ b2 ^ b3 ^ b5);
    auto p2 = (b0 ^ b1 ^ b3 ^ b6);
    
    auto parity = (p0 << 0) | (p1 << 1) | (p2 << 2);
    if (parity != 0) error = true;
    switch (parity)
    {
        case 0x5: return (b ^ 1) & 0xf;
        case 0x7: return (b ^ 2) & 0xf;
        case 0x3: return (b ^ 4) & 0xf;
        case 0x6: return (b ^ 8) & 0xf;
        case 0x0:
        case 0x1:
        case 0x2:
        case 0x4: return b & 0xF;
    }
    return b & 0xf;
}

/***********************************************************************
 * Check parity for 5/4 code.
 * return true if parity is valid.
 **********************************************************************/
static inline unsigned char checkParity54(const unsigned char b, bool &error) {
	auto x = b ^ (b >> 2);
	x = x ^ (x >> 1) ^ (b >> 4);
	if (x & 1) error = true;
	return b & 0xf;
}

static inline unsigned char encodeParity54(const unsigned char b) {
	auto x = b ^ (b >> 2);
	x = x ^ (x >> 1);
	return (b & 0xf) | ((x << 4) & 0x10);
}

/***********************************************************************
* Check parity for 6/4 code.
* return true if parity is valid.
**********************************************************************/
static inline unsigned char checkParity64(const unsigned char b, bool &error) {
	auto x = b ^ (b >> 1) ^ (b >> 2);
	auto y = x ^ b ^ (b >> 3);
	
	x ^= b >> 4;
	y ^= b >> 5;
	if ((x | y) & 1) error = true;
	return b & 0xf;
}

static inline unsigned char encodeParity64(const unsigned char b) {
	auto x = b ^ (b >> 1) ^ (b >> 2);
	auto y = x ^ b ^ (b >> 3);
	return ((x & 1) << 4) | ((y & 1) << 5) | (b & 0xf);
}

/***********************************************************************
 * Diagonal interleaver + deinterleaver
 **********************************************************************/
static inline void diagonalInterleaveSx(const uint8_t *codewords, const size_t numCodewords, uint16_t *symbols, const size_t PPM, const size_t RDD){
	for (size_t x = 0; x < numCodewords / PPM; x++)	{
		const size_t cwOff = x*PPM;
		const size_t symOff = x*(4 + RDD);
		for (size_t k = 0; k < 4 + RDD; k++){
			for (size_t m = 0; m < PPM; m++){
				const size_t i = (m + k + PPM) % PPM;
				const auto bit = (codewords[cwOff + i] >> k) & 0x1;
				symbols[symOff + k] |= (bit << m);
			}
		}
	}
}

static inline void diagonalDeterleaveSx(const uint16_t *symbols, const size_t numSymbols, uint8_t *codewords, const size_t PPM, const size_t RDD)
{
	for (size_t x = 0; x < numSymbols / (4 + RDD); x++)
	{
		const size_t cwOff = x*PPM;
		const size_t symOff = x*(4 + RDD);
		for (size_t k = 0; k < 4 + RDD; k++)
		{
			for (size_t m = 0; m < PPM; m++)
			{
				const size_t i = (m + k) % PPM;
				const auto bit = (symbols[symOff + k] >> m) & 0x1;
				codewords[cwOff + i] |= (bit << k);
			}
		}
	}
}

static inline void diagonalDeterleaveSx2(const uint16_t *symbols, const size_t numSymbols, uint8_t *codewords, const size_t PPM, const size_t RDD){
	size_t nb = RDD + 4;
	for (size_t x = 0; x < numSymbols / nb; x++) {
		const size_t cwOff = x*PPM;
		const size_t symOff = x*nb;
		for (size_t m = 0; m < PPM; m++) {
			size_t i = m;
			auto sym = symbols[symOff + m];
			for (size_t k = 0; k < PPM; k++, sym >>= 1) {
				codewords[cwOff + i] |= (sym & 1) << m;
				if (++i == PPM) i = 0;
			}
		}
	}
}
