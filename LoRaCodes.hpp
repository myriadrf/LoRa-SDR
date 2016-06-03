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
 * https://en.wikipedia.org/wiki/Hamming_code
 **********************************************************************/
static inline unsigned char encodeHamming84(const unsigned char x)
{
    auto d0 = (x >> 0) & 0x1;
    auto d1 = (x >> 1) & 0x1;
    auto d2 = (x >> 2) & 0x1;
    auto d3 = (x >> 3) & 0x1;

    unsigned char b = 0;
    b |= ((d0 + d1 + d3) & 0x1) << 0;
    b |= ((d0 + d2 + d3) & 0x1) << 1;
    b |= ((d0) & 0x1) << 2;
    b |= ((d1 + d2 + d3) & 0x1) << 3;
    b |= ((d1) & 0x1) << 4;
    b |= ((d2) & 0x1) << 5;
    b |= ((d3) & 0x1) << 6;
    b |= ((d0 + d1 + d2) & 0x1) << 7;
    return b;
}

/***********************************************************************
 * Decode 8 bits into a 4 bit word with single bit correction
 * Set error true when the result is known to be in error
 **********************************************************************/
static inline unsigned char decodeHamming84(const unsigned char b, bool &error)
{
    auto b0 = (b >> 0) & 0x1;
    auto b1 = (b >> 1) & 0x1;
    auto b2 = (b >> 2) & 0x1;
    auto b3 = (b >> 3) & 0x1;
    auto b4 = (b >> 4) & 0x1;
    auto b5 = (b >> 5) & 0x1;
    auto b6 = (b >> 6) & 0x1;
    auto b7 = (b >> 7) & 0x1;

    auto p0 = (b0 + b2 + b4 + b6) & 0x1;
    auto p1 = (b1 + b2 + b5 + b6) & 0x1;
    auto p2 = (b3 + b4 + b5 + b6) & 0x1;
    auto p3 = (b0 + b1 + b2 + b3 + b4 + b5 + b6 + b7) & 0x1;

    auto parity = (p0 << 0) | (p1 << 1) | (p2 << 2) | (p3 << 3);
    switch (parity & 0xf)
    {
    case 0: break; //no error
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7: error = true; break; //uncorrectable
    case 8: b7 = (~b7) & 0x1; break;
    case 9: b0 = (~b0) & 0x1; break;
    case 10: b1 = (~b1) & 0x1; break;
    case 11: b2 = (~b2) & 0x1; break;
    case 12: b3 = (~b3) & 0x1; break;
    case 13: b4 = (~b4) & 0x1; break;
    case 14: b5 = (~b5) & 0x1; break;
    case 15: b6 = (~b6) & 0x1; break;
    }

    return (b2 << 0) | (b4 << 1) | (b5 << 2) | (b6 << 3);
}
