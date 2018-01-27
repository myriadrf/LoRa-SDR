########################################################################
# Project setup
########################################################################
cmake_minimum_required(VERSION 2.8.9)
project(LoRa_Blocks CXX)

find_package(Pothos "0.6" CONFIG REQUIRED)

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
# json.hpp header
########################################################################
find_path(JSON_HPP_INCLUDE_DIR NAMES json.hpp PATH_SUFFIXES nlohmann)

if (NOT JSON_HPP_INCLUDE_DIR)
    message(FATAL_ERROR "LoRa toolkit requires json.hpp, skipping...")
endif (NOT JSON_HPP_INCLUDE_DIR)

include_directories(${JSON_HPP_INCLUDE_DIR})

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
        TestLoopback.cpp
        TestGen.cpp
        BlockGen.cpp
        TestCodesSx.cpp
        TestDetector.cpp
    DESTINATION lora
    ENABLE_DOCS
)
