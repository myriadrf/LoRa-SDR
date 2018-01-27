/***********************************************************************
 * least error decoder
 * The symbols have +/-1 error due to chirp gray encode
 * Calculate error for each interleaved section hamming/parity decode
 * for different symbol adjustments until the minimum error is found.
 **********************************************************************/

#include "LoRaCodes.hpp"
#include <iostream>
#include <vector>
#include <iomanip>

#define MAX_NUM_PERTURBATIONS 256 //2**8
static uint8_t PERTURBATIONS_TABLE[MAX_NUM_PERTURBATIONS][8];

struct InitPerturbations
{
    InitPerturbations(void)
    {
        for (size_t i = 0; i < MAX_NUM_PERTURBATIONS; i++)
        {
            for (size_t j = 0; j < 8; j++)
            {
                PERTURBATIONS_TABLE[i][j] = (i/(1 << j))%2;
                //std::cout << std::setw(2) << int(PERTURBATIONS_TABLE[i][j]) << " ";
            }
            //std::cout << std::endl;
        }
    }
};

static inline void applyPerturbation(uint16_t *symbolsOut, const uint16_t *symbolsIn, const size_t nb, const int tableIndex)
{
    for (size_t i = 0; i < nb; i++)
    {
        symbolsOut[i] = binaryToGray16(symbolsIn[i] + PERTURBATIONS_TABLE[tableIndex][i]);
    }
}

static inline void perturbSymbolBlock(uint16_t *symbols, const size_t ppm, const size_t rrd, const uint8_t *whiten_mask)
{
    size_t nb = rrd + 4;
    //std::cout << "perturbSymbolBlock nb=" << nb << std::endl;
    size_t errorCounts[MAX_NUM_PERTURBATIONS];
    size_t leastErrorIndex = 0;
    size_t numPerturbations = 1 << (rrd+4);
    /*
    std::vector<uint16_t> newSymbols(nb);
    std::vector<uint8_t> codewords(ppm);
    for (size_t i = 0; i < nb; i++) newSymbols[i] = binaryToGray16(symbols[i]);
    diagonalDeterleaveSx(newSymbols.data(), nb, codewords.data(), ppm, rrd);
    for (size_t j = 0; j < ppm; j++) codewords[j] ^= whiten_mask[j];
    for (size_t i = 0; i < ppm ; i++) std::cout << std::setw(2) << std::hex << int(codewords[i]) << " ";
    */

    for (size_t i = 0; i < numPerturbations; i++)
    {
        std::vector<uint16_t> newSymbols(nb);
        std::vector<uint8_t> codewords(ppm);
        size_t newErrorCount(0);
        applyPerturbation(newSymbols.data(), symbols, nb, i);
        diagonalDeterleaveSx(newSymbols.data(), nb, codewords.data(), ppm, rrd);
        for (size_t j = 0; j < ppm; j++) codewords[j] ^= whiten_mask[j];
        switch (rrd)
        {
        case 1: for (size_t j = 0; j < ppm; j++)
        {
            bool error = false;
            checkParity54(codewords[j], error);
            if (error) newErrorCount++;
        } break;
        case 2: for (size_t j = 0; j < ppm; j++)
        {
            bool error = false;
            checkParity64(codewords[j], error);
            if (error) newErrorCount++;
        } break;
        case 3: for (size_t j = 0; j < ppm; j++)
        {
            bool error = false;
            decodeHamming74sx(codewords[j], error);
            if (error) newErrorCount++;
        } break;
        case 4: for (size_t j = 0; j < ppm; j++)
        {
            bool error = false;
            bool bad = false;
            decodeHamming84sx(codewords[j], error, bad);
            if (error || bad) newErrorCount++;
        } break;
        }
        errorCounts[i] = newErrorCount;
        if (errorCounts[leastErrorIndex] > newErrorCount) leastErrorIndex = i;
        if (newErrorCount == 0) break;
    }
    /*
    if (leastErrorIndex != 0)
    {
        std::cout << "---> leastErrorIndex " << leastErrorIndex << ", errorCounts[leastErrorIndex]=" << errorCounts[leastErrorIndex] << std::endl;
    }
    //*/
    applyPerturbation(symbols, symbols, nb, leastErrorIndex); //output
}

void leastErrorDecoder(
    uint16_t *symbols, const size_t numSymbols,
    const size_t ppm, const size_t rrd)
{
    if (rrd == 0) return; //not useful, no errors to detect (could kind of work with checksum though)
    static InitPerturbations initPerturbations;
    //std::cout << "numSymbols " << numSymbols << ", ppm " << ppm << ", rrd " << rrd << std::endl;
    size_t nb = rrd + 4;
    std::vector<uint8_t> whiten_mask((numSymbols*ppm)/nb, 0);
    Sx1272ComputeWhiteningLfsr(whiten_mask.data()+N_HEADER_CODEWORDS, whiten_mask.size()-N_HEADER_CODEWORDS, 0, rrd);
    /*
    for (size_t i = 0; i < whiten_mask.size() ; i++)
    {
        std::cout << std::hex << int(whiten_mask[i]) << " ";
    }
    std::cout << std::endl;
    //*/
    size_t symOfs(0);
    size_t cwOfs(0);
    //std::cout << "-< ";
    for (size_t x = 0; x < numSymbols / nb; x++) //for each interleaved block
    {
        perturbSymbolBlock(symbols+symOfs, ppm, rrd, whiten_mask.data()+cwOfs);
        cwOfs += ppm;
        symOfs += nb;
    }
    //std::cout << std::endl;
}
