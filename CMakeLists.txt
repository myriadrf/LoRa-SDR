########################################################################
# Project setup
########################################################################
cmake_minimum_required(VERSION 2.8.9)
project(LoRa_Blocks CXX)

find_package(Pothos CONFIG REQUIRED)

########################################################################
## Compiler specifics
########################################################################
enable_language(C)
include(CheckIncludeFiles)
CHECK_INCLUDE_FILES(alloca.h HAS_ALLOCA_H)
if(HAS_ALLOCA_H)
    add_definitions(-DHAS_ALLOCA_H)
endif(HAS_ALLOCA_H)

########################################################################
## LoRa blocks
########################################################################
POTHOS_MODULE_UTIL(
    TARGET LoRa_Blocks
    SOURCES
        LoRaDemod.cpp
        LoRaMod.cpp
        LoRaEncoder.cpp
        LoRaDecoder.cpp
        LoRaTest.cpp
        TestGen.cpp
		BlockGen.cpp
        TestCodesSx.cpp
    DESTINATION lora
    ENABLE_DOCS
)
