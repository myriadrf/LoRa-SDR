########################################################################
# Project setup
########################################################################
cmake_minimum_required(VERSION 2.8.9)
project(LoRa_Blocks CXX)

find_package(Pothos CONFIG REQUIRED)

########################################################################
## BTLE blocks
########################################################################
POTHOS_MODULE_UTIL(
    TARGET LoRa_Blocks
    SOURCES
        LoRaDemod.cpp
    DESTINATION lora
    ENABLE_DOCS
)
