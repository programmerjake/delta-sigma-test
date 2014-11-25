#include <cstdlib>
#include <cassert>
#include <iostream>
#include <string>
#include <stdint.h>
#include <cmath>
#include <termios.h>
#include <cstdio>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <signal.h>
#include <cerrno>
#include <fstream>

#if 1
#define BAUD_RATE 460800
#define BAUD_RATE_B B460800
#else
#define BAUD_RATE 921600
#define BAUD_RATE_B B921600
#endif

using namespace std;

void error(string msg)
{
    cerr << "error : " << msg << endl;
    exit(1);
}

inline float limit(float v, float minimum, float maximum)
{
    return v > maximum ? maximum : v < minimum ? minimum : v;
}

class SigmaDeltaModulator
{
    float sum1, sum2;
public:
    SigmaDeltaModulator()
        : sum1(0), sum2(0)
    {
    }
    bool forcedStep(float input, bool retval)
    {
        const float factor1 = 0.5f, factor2 = 0.5f;
        input = limit(input, -1, 1);
        float feedback = retval ? 1 : -1;
        sum1 += (input - feedback) * factor1;
        sum2 += (sum1 - feedback) * factor2;
        return retval;
    }
    bool step(float input)
    {
        return forcedStep(input, sum2 > 0);
    }
};

class SerialOutput
{
    FILE *f;
    SerialOutput(const SerialOutput &); // not implemented
    void operator =(const SerialOutput &); // not implemented
public:
    SerialOutput(string devicePath)
    {
        signal(SIGIO, SIG_IGN);
        int fd = open(devicePath.c_str(), O_NOCTTY | O_WRONLY);
        if(fd == -1)
            error("can't open : " + devicePath);
        termios settings;
        if(0 != tcgetattr(fd, &settings))
            error("can't get settings : " + devicePath);
        cfmakeraw(&settings);
        cfsetispeed(&settings, BAUD_RATE_B);
        cfsetospeed(&settings, BAUD_RATE_B);
        if(0 != tcsetattr(fd, TCSANOW, &settings))
            error("can't get settings : " + devicePath);
        f = fdopen(fd, "wb");
        if(!f)
            error("can't run fdopen");
    }
    void write(uint8_t v)
    {
        fputc(v, f);
    }
    ~SerialOutput()
    {
        fclose(f);
    }
};

class SignalSourceFile
{
    double sampleRate;
    int bytesPerSample;
    ifstream f;
    float lastSample, currentSample;
public:
    bool done;
private:
    float fractSample;
    bool isSigned;
    SignalSourceFile(const SignalSourceFile &); // not implemented
    void operator =(const SignalSourceFile &); // not implemented
public:
    SignalSourceFile(string fileName, double sampleRate, int bytesPerSample, bool isSigned)
        : sampleRate(sampleRate), bytesPerSample(bytesPerSample), done(false), fractSample(0), isSigned(isSigned)
    {
        f.open(fileName.c_str(), ios::binary | ios::in);
        if(!f)
            error("can't open file : " + fileName);
        lastSample = readSample();
        currentSample = readSample();
    }
    ~SignalSourceFile()
    {
    }
    float readSample()
    {
        if(done)
            return 0;
        int b = f.get();
        if(b == EOF)
        {
            done = true;
            return 0;
        }
        int32_t minValue = -0x80 << 8 * (bytesPerSample - 1);
        int32_t value = b;
        int32_t mask = -0x100 << 8 * (bytesPerSample - 1);
        for(int i = 1; i < bytesPerSample; i++)
        {
            b = f.get();
            if(b == EOF)
            {
                done = true;
                return 0;
            }
            value |= (int32_t)b << 8 * i;
        }
        if(!isSigned)
        {
            return (float)(int64_t)(uint32_t)value / -(float)minValue - 1;
        }
        if(value & minValue)
            value |= mask;
        return -(float)value / minValue;
    }
    float operator ()(float deltaTime)
    {
        fractSample += deltaTime * sampleRate;
        float floorFractSample = floor(fractSample);
        int readCount = (int)floorFractSample;
        fractSample -= floorFractSample;
        while(readCount-- > 0)
        {
            lastSample = currentSample;
            currentSample = readSample();
        }
        return lastSample + fractSample * (currentSample - lastSample);
    }
};

class SignalSourceSine
{
    float angle;
    float frequency, amplitude;
public:
    bool done;
    SignalSourceSine(float frequency, float amplitude)
        : angle(0), frequency(frequency), amplitude(amplitude), done(false)
    {
    }
    float operator ()(float deltaTime)
    {
        angle += 2 * M_PI * frequency * deltaTime;
        angle = fmod(angle, 2 * M_PI);
        return amplitude * sin(angle);
    }
};
#if 1
int main()
{
    SigmaDeltaModulator sdm;
#if 1
    SignalSourceFile signalSource("audio-data-16le-44100-mono.bin", 44100, 2, true);
#else
    SignalSourceSine signalSource(440, 1);
#endif
    SerialOutput serialOutput("/dev/ttyUSB0");
    int bitNumber = 0;
    uint8_t byteValue = 0;
    size_t currentBitCount = 0;
    while(!signalSource.done)
    {
        float signal = signalSource(1.0f / BAUD_RATE);
        bool sdmOutput;
        const int startBitCount = 1, stopBitCount = 1, dataBitCount = 8;
        signal *= dataBitCount / (double)(startBitCount + stopBitCount + dataBitCount);
        bitNumber++;
        bitNumber %= startBitCount + stopBitCount + dataBitCount;
        if(bitNumber < startBitCount)
            sdmOutput = sdm.forcedStep(signal, false);
        else if(bitNumber < startBitCount + dataBitCount)
        {
            sdmOutput = sdm.step(signal);
            byteValue >>= 1;
            if(sdmOutput)
                byteValue |= 0x80;
            if(++currentBitCount >= dataBitCount)
            {
                currentBitCount = 0;
                serialOutput.write(byteValue >> (8 - dataBitCount));
            }
        }
        else
            sdmOutput = sdm.forcedStep(signal, true);
    }
    return 0;
}
#else
int main()
{
    double frame = 0;
    while(true)
    {
        frame += 1.0f;
        float signal = sin(floor(frame / 200) / 20 * 2 * M_PI);
        signal = floor(signal * 1e7 + 0.5f) / 1e7;
        signal *= 0.8f;
        cout << "\x1b[H" << signal << "\x1b[K\n";
        SigmaDeltaModulator sdm;
        sdm.step(1);
        for(int i = 0; i < 5; i++)
            sdm.step(signal);
        for(int y = 0; y < 20; y++)
        {
            for(int x = 0; x < 70; x++)
            {
                bool v;
                if(x % 10 < 2)
                    v = sdm.forcedStep(signal, x % 10 == 0);
                else
                    v = sdm.step(signal);
                if(v)
                    cout << "#";
                else
                    cout << " ";
            }
            cout << "\x1b[K\n";
        }
        cout << flush;
    }
    return 0;
}
#endif
