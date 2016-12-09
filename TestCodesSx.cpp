// Copyright (c) 2016-2016 Lime Microsystems
// SPDX-License-Identifier: BSL-1.0

#include <Pothos/Testing.hpp>
#include <iostream>
#include "LoRaCodes.hpp"

POTHOS_TEST_BLOCK("/lora/tests", test_hamming84_sx)
{
    bool error;
    bool bad;
    unsigned char decoded;

    //test hamming 84 with bit errors
    for (size_t i = 0; i < 16; i++)
    {
        unsigned char byte = i & 0xff;
        unsigned char encoded = encodeHamming84sx(byte);

        //check no bit errors
        error = false;
        bad = false;
        decoded = decodeHamming84sx(encoded, error, bad);
        POTHOS_TEST_TRUE(not error);
        POTHOS_TEST_TRUE(not bad);
        POTHOS_TEST_EQUAL(byte, decoded);

        for (int bit0 = 0; bit0 < 8; bit0++)
        {
            //check 1 bit error
            error = false;
            bad = false;
            unsigned char encoded1err = encoded ^ (1 << bit0);
            decoded = decodeHamming84sx(encoded1err, error, bad);
            POTHOS_TEST_TRUE(error);
            POTHOS_TEST_TRUE(not bad);
            POTHOS_TEST_EQUAL(byte, decoded);

            for (int bit1 = 0; bit1 < 8; bit1++)
            {
                if (bit1 == bit0) continue;

                //check 2 bit errors (cant correct, but can detect
                error = false;
                bad = false;
                unsigned char encoded2err = encoded1err ^ (1 << bit1);
                decoded = decodeHamming84sx(encoded2err, error, bad);
                POTHOS_TEST_TRUE(error);
                POTHOS_TEST_TRUE(bad);
            }
        }
    }
}

POTHOS_TEST_BLOCK("/lora/tests", test_hamming74_sx)
{
    bool error;
    unsigned char decoded;

    //test hamming 74 with bit errors
    for (size_t i = 0; i < 16; i++)
    {
        unsigned char byte = i & 0xff;
        unsigned char encoded = encodeHamming74sx(byte);

        //check no bit errors
        error = false;
        decoded = decodeHamming74sx(encoded, error);
        POTHOS_TEST_TRUE(not error);
        POTHOS_TEST_EQUAL(byte, decoded);

        for (int bit0 = 0; bit0 < 7; bit0++)
        {
            //check 1 bit error
            error = false;
            unsigned char encoded1err = encoded ^ (1 << bit0);
            decoded = decodeHamming74sx(encoded1err, error);
            POTHOS_TEST_TRUE(error);
            POTHOS_TEST_EQUAL(byte, decoded);
        }
    }
}

POTHOS_TEST_BLOCK("/lora/tests", test_parity64_sx)
{
    bool error;
    unsigned char decoded;

    //test parity 64, see if bit error detected
    for (size_t i = 0; i < 16; i++)
    {
        unsigned char byte = i & 0xff;
        unsigned char encoded = encodeParity64(byte);

        //check no bit errors
        error = false;
        decoded = checkParity64(encoded, error);
        POTHOS_TEST_TRUE(not error);
        POTHOS_TEST_EQUAL(byte, decoded);

        for (int bit0 = 0; bit0 < 8; bit0++)
        {
            //check 1 bit error
            unsigned char encoded1err = encoded ^ (1 << bit0);
            decoded = checkParity64(encoded1err, error);
            POTHOS_TEST_TRUE(error);
        }
    }
}

POTHOS_TEST_BLOCK("/lora/tests", test_parity54_sx)
{
    bool error;
    unsigned char decoded;

    //test parity 54, see if bit error detected
    for (size_t i = 0; i < 16; i++)
    {
        unsigned char byte = i & 0xff;
        unsigned char encoded = encodeParity54(byte);

        //check no bit errors
        error = false;
        decoded = checkParity54(encoded, error);
        POTHOS_TEST_TRUE(not error);
        POTHOS_TEST_EQUAL(byte, decoded);

        for (int bit0 = 0; bit0 < 8; bit0++)
        {
            //check 1 bit error
            unsigned char encoded1err = encoded ^ (1 << bit0);
            decoded = checkParity54(encoded1err, error);
            POTHOS_TEST_TRUE(error);
        }
    }
}

POTHOS_TEST_BLOCK("/lora/tests", test_interleaver_sx)
{
    for (size_t PPM = 7; PPM <= 12; PPM++)
    {
        std::cout << "Testing PPM " << PPM << std::endl;
        for (size_t RDD = 0; RDD <= 4; RDD++)
        {
            std::cout << "  with RDD " << RDD << std::endl;
            std::vector<uint8_t> inputCws(PPM);
            const auto mask = (1 << (RDD+4))-1;
            for (auto &x : inputCws) x = std::rand() & mask;

            std::vector<uint16_t> symbols(((RDD+4)*inputCws.size())/PPM);
            diagonalInterleaveSx(inputCws.data(), inputCws.size(), symbols.data(), PPM, RDD);

            std::vector<uint8_t> outputCws(inputCws.size());
            diagonalDeterleaveSx(symbols.data(), symbols.size(), outputCws.data(), PPM, RDD);

            POTHOS_TEST_EQUALV(inputCws, outputCws);
        }
    }
}
