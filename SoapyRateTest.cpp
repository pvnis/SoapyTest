// Copyright (c) 2016-2017 Josh Blum
// SPDX-License-Identifier: BSL-1.0

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Errors.hpp>
#include <string>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <csignal>
#include <chrono>
#include <cstdio>
#include <thread>

static sig_atomic_t loopDone = false;
static void sigIntHandler(const int)
{
    loopDone = true;
}

void runRateTestStreamLoop(
    SoapySDR::Device *device,
    SoapySDR::Stream *stream,
    const int direction,
    const size_t numChans,
    const size_t elemSize)
{
    //allocate buffers for the stream read/write
    const size_t numElems = device->getStreamMTU(stream);
    std::vector<std::vector<char>> buffMem(numChans, std::vector<char>(elemSize*numElems));
    std::vector<void *> buffs(numChans);
    for (size_t i = 0; i < numChans; i++) buffs[i] = buffMem[i].data();

    //state collected in this loop
    unsigned int overflows(0);
    unsigned int underflows(0);
    unsigned long long totalSamples(0);
    const auto startTime = std::chrono::high_resolution_clock::now();
    auto timeLastPrint = std::chrono::high_resolution_clock::now();
    auto timeLastSpin = std::chrono::high_resolution_clock::now();
    auto timeLastStatus = std::chrono::high_resolution_clock::now();
    int spinIndex(0);

    std::cout << "Starting stream " << direction << std::endl;
    device->activateStream(stream);
    
    while (not loopDone)
    {
        int ret(0);
        int flags(0);
        long long timeNs(0);
        switch(direction)
        {
        case SOAPY_SDR_RX:
            //std::cout << "Read stream " << std::endl;
            ret = device->readStream(stream, buffs.data(), numElems, flags, timeNs);
            break;
        case SOAPY_SDR_TX:
            //std::cout << "Write stream " << std::endl;
            ret = device->writeStream(stream, buffs.data(), numElems, flags, timeNs);
            break;
        }

        if (ret == SOAPY_SDR_TIMEOUT) continue;
        if (ret == SOAPY_SDR_OVERFLOW)
        {
            overflows++;
            continue;
        }
        if (ret == SOAPY_SDR_UNDERFLOW)
        {
            underflows++;
            continue;
        }
        if (ret < 0)
        {
            std::cerr << "Unexpected stream error " << SoapySDR::errToStr(ret) << std::endl;
            break;
        }
        totalSamples += ret;

        const auto now = std::chrono::high_resolution_clock::now();
        if (timeLastSpin + std::chrono::milliseconds(300) < now)
        {
            timeLastSpin = now;
            static const char spin[] = {"|/-\\"};
            printf("\b%c", spin[(spinIndex++)%4]);
            fflush(stdout);
        }
        //occasionally read out the stream status (non blocking)
        if (timeLastStatus + std::chrono::seconds(1) < now)
        {
            timeLastStatus = now;
            while (true)
            {
                size_t chanMask; int flags; long long timeNs;
                ret = device->readStreamStatus(stream, chanMask, flags, timeNs, 0);
                if (ret == SOAPY_SDR_OVERFLOW) overflows++;
                else if (ret == SOAPY_SDR_UNDERFLOW) underflows++;
                else if (ret == SOAPY_SDR_TIME_ERROR) {}
                else break;
            }
        }
        if (timeLastPrint + std::chrono::seconds(5) < now)
        {
            timeLastPrint = now;
            const auto timePassed = std::chrono::duration_cast<std::chrono::microseconds>(now - startTime);
            const auto sampleRate = double(totalSamples)/timePassed.count();
            printf("\b%g Msps\t%g MBps - Dir %d", sampleRate, sampleRate*numChans*elemSize, direction);
            if (overflows != 0) printf("\tOverflows %u", overflows);
            if (underflows != 0) printf("\tUnderflows %u", underflows);
            printf("\n ");
        }

    }
    device->deactivateStream(stream);
}

int SoapySDRRateTest(
    const std::string &argStr,
    const double frequency,
    const double sampleRate,
    const std::string &formatStr,
    const std::string &channelStr)
{
    SoapySDR::Device *device(nullptr);

    try
    {
        device = SoapySDR::Device::make(argStr);

        //build channels list, using KwargsFromString is a easy parsing hack
        std::vector<size_t> channels;
        for (const auto &pair : SoapySDR::KwargsFromString(channelStr))
        {
            channels.push_back(std::stoi(pair.first));
        }
        if (channels.empty()) channels.push_back(0);

        //initialize the frequency and sample rate for all channels
        for (const auto &chan : channels)
        {
            device->setFrequency(SOAPY_SDR_RX, chan, frequency);
            device->setFrequency(SOAPY_SDR_TX, chan, frequency);
            
            device->setSampleRate(SOAPY_SDR_RX, chan, sampleRate);
            device->setSampleRate(SOAPY_SDR_TX, chan, sampleRate);
        }

        //create the stream, use the native format
        double fullScale(0.0);
        const auto rxFormat = formatStr.empty() ? device->getNativeStreamFormat(SOAPY_SDR_RX, channels.front(), fullScale) : formatStr;
        const size_t rxElemSize = SoapySDR::formatToSize(rxFormat);
        auto rxStream = device->setupStream(SOAPY_SDR_RX, rxFormat, channels);

        const auto txFormat = formatStr.empty() ? device->getNativeStreamFormat(SOAPY_SDR_RX, channels.front(), fullScale) : formatStr;
        const size_t txElemSize = SoapySDR::formatToSize(txFormat);
        auto txStream = device->setupStream(SOAPY_SDR_TX, txFormat, channels);

        //run the rate test one setup is complete
        std::cout << "RX format: " << rxFormat << " TX format: " << txFormat << std::endl;
        std::cout << "Num channels: " << channels.size() << std::endl;
        std::cout << "RX Element size: " << rxElemSize << " bytes" << "TX Element size: " << txElemSize << " bytes" << std::endl;
        std::cout << "Begin rate test at " << (sampleRate/1e6) << " Msps" << std::endl;
        //runRateTestStreamLoop(device, stream, direction, channels.size(), elemSize);

        signal(SIGINT, sigIntHandler);

        std::cout << "Create rxThread " << std::endl;
        auto rxThread = std::thread([device, rxStream, channels, rxElemSize]() {
            runRateTestStreamLoop(device, rxStream, SOAPY_SDR_RX, channels.size(), rxElemSize);
        });

        sleep(2);
        
        std::cout << "Create txThread " << std::endl;
        auto txThread = std::thread([device, txStream, channels, txElemSize]() {
            runRateTestStreamLoop(device, txStream, SOAPY_SDR_TX, channels.size(), txElemSize);
        });

        std::cout << "Join rxThread " << std::endl;
        rxThread.join();

        //cleanup stream and device
        device->closeStream(rxStream);
        device->closeStream(txStream);
        SoapySDR::Device::unmake(device);
    }
    catch (const std::exception &ex)
    {
        std::cerr << "Error in rate test: " << ex.what() << std::endl;
        SoapySDR::Device::unmake(device);
        return EXIT_FAILURE;
    }
    return EXIT_FAILURE;
}
