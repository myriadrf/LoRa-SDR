/***********************************************************************
 * least error decoder
 * The symbols have +/-1 error due to chirp gray encode
 * Calculate error for each interleaved section hamming/parity decode
 * for different symbol adjustments until the minimum error is found.
 **********************************************************************/

#include "LoRaCodes.hpp"
#include <iostream>
#include <iomanip>

#define MAX_NUM_PERTURBATIONS 6561 //3**8
static signed char PERTURBATIONS_TABLE[MAX_NUM_PERTURBATIONS][8];
static size_t RRD_TO_PERTURBATIONS[5];

int64_t ipow(int64_t base, int exp){
  int64_t result = 1;
  while(exp){
    if(exp & 1)
      result *= base;
    exp >>= 1;
    base *= base;
  }
  return result;
}

struct InitPerturbations
{
    InitPerturbations(void)
    {
        for (size_t i = 0; i < MAX_NUM_PERTURBATIONS; i++)
        {
            for (size_t j = 0; j < 8; j++)
            {
                int num = (i/ipow(3, j))%3;
                PERTURBATIONS_TABLE[i][j] = (num==2)?-1:num;
            }
        }
        for (size_t rrd = 0; rrd <= 4; rrd++)
        {
            RRD_TO_PERTURBATIONS[rrd] = ipow(3, rrd+4);
        }
        /*
        for (size_t i = 0; i < MAX_NUM_PERTURBATIONS; i++)
        {
            for (size_t j = 0; j < 8; j++)
                std::cout << std::setw(2) << int(PERTURBATIONS_TABLE[i][j]) << " ";
            std::cout << std::endl;
        }
        */
    }
};

static inline void applyPerturbation(uint16_t *symbolsOut, const uint16_t *symbolsIn, const size_t nb, const int tableIndex)
{
    for (size_t i = 0; i < nb; i++)
    {
        symbolsOut[i] = (symbolsIn[i] + PERTURBATIONS_TABLE[tableIndex][i]) & 0xff;
    }
}

static inline void perturbSymbolBlock(uint16_t *symbols, const size_t cwOfs, const size_t bitOfs, const size_t ppm, const size_t rrd)
{
    size_t nb = rrd + 4;
    std::cout << "perturbSymbolBlock nb=" << nb << std::endl;
    size_t errorCounts[MAX_NUM_PERTURBATIONS];
    size_t leastErrorIndex = 0;
    size_t numPerturbations = RRD_TO_PERTURBATIONS[rrd];
    uint16_t newSymbols[8];
    uint8_t codewords[12];
    for (size_t i = 0; i < numPerturbations; i++)
    {
        size_t newErrorCount(0);
        applyPerturbation(newSymbols, symbols, nb, i);
        diagonalDeterleaveSx(newSymbols, nb, codewords, ppm, rrd);
        Sx1272ComputeWhiteningLfsr(codewords+cwOfs, ppm-cwOfs, bitOfs, rrd);
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
    }
    if (leastErrorIndex != 0)
    {
        std::cout << "---> leastErrorIndex " << leastErrorIndex << std::endl;
    }
    //applyPerturbation(symbols, symbols, nb, leastErrorIndex); //output
}

void leastErrorDecoder(
    uint16_t *symbols, const size_t numSymbols,
    const size_t cwOfs, const size_t bitOfs,
    const size_t ppm, const size_t rrd)
{
    if (rrd == 0) return; //not useful, no errors to detect (could kind of work with checksum though)
    static InitPerturbations initPerturbations;
    std::cout << "numSymbols " << numSymbols << ", ppm " << ppm << ", rrd " << rrd << std::endl;
    size_t nb = rrd + 4;
    for (size_t x = 0; x < numSymbols / nb; x++) //for each interleaved block
    {
        perturbSymbolBlock(symbols + x*nb, cwOfs, bitOfs, ppm, rrd);
    }
}
