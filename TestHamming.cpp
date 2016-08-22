// Copyright (c) 2016-2016 Lime Microsystems
// SPDX-License-Identifier: BSL-1.0

#include <Pothos/Testing.hpp>
#include <iostream>
#include "LoRaCodes.hpp"

POTHOS_TEST_BLOCK("/lora/tests", test_hamming84)
{
    bool error;
    unsigned char decoded;

    //test hamming 84 with bit errors
    for (size_t i = 0; i < 16; i++)
    {
        unsigned char byte = i & 0xff;
        unsigned char encoded = encodeHamming84sx(byte);

        //check no bit errors
        error = false;
        decoded = decodeHamming84sx(encoded, error);
        POTHOS_TEST_TRUE(not error);
        POTHOS_TEST_EQUAL(byte, decoded);

        for (int bit0 = 0; bit0 < 8; bit0++)
        {
            //check 1 bit error
            error = false;
            unsigned char encoded1err = encoded ^ (1 << bit0);
            decoded = decodeHamming84sx(encoded1err, error);
            POTHOS_TEST_TRUE(not error);
            POTHOS_TEST_EQUAL(byte, decoded);

            for (int bit1 = 0; bit1 < 8; bit1++)
            {
                if (bit1 == bit0) continue;

                //check 2 bit errors (cant correct, but can detect
                error = false;
                unsigned char encoded2err = encoded1err ^ (1 << bit1);
                decoded = decodeHamming84sx(encoded2err, error);
                POTHOS_TEST_TRUE(error);
            }
        }
    }
}

POTHOS_TEST_BLOCK("/lora/tests", test_hamming74)
{
    bool error;
    unsigned char decoded;

    //test hamming 74 with bit errors
    for (size_t i = 0; i < 16; i++)
    {
        unsigned char byte = i & 0xff;
        unsigned char encoded = encodeHamming74sx(byte);

        //check no bit errors
        decoded = decodeHamming74sx(encoded);
        POTHOS_TEST_EQUAL(byte, decoded);

        for (int bit0 = 0; bit0 < 8; bit0++)
        {
            //check 1 bit error
            unsigned char encoded1err = encoded ^ (1 << bit0);
            decoded = decodeHamming74sx(encoded1err);
            POTHOS_TEST_EQUAL(byte, decoded);
        }
    }
}
