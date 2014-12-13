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
#include <vector>
#include <ctime>
#include <linux/serial.h>
#include <sys/ioctl.h>

#define USE_VORBISFILE

#ifdef USE_VORBISFILE
#include <vorbis/vorbisfile.h>
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
#if 1
class SigmaDeltaModulator
{
    float sum;
public:
    SigmaDeltaModulator()
        : sum(0)
    {
    }
    bool forcedStep(float input, bool retval)
    {
        input = limit(input, -1, 1);
        float feedback = retval ? 1 : -1;
        sum += input - feedback;
        return retval;
    }
    bool step(float input)
    {
        return forcedStep(input, sum > 0);
    }
};
#else
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
#endif

class SerialOutput
{
    FILE *f;
    float actualBaudRate;
    SerialOutput(const SerialOutput &); // not implemented
    void operator =(const SerialOutput &); // not implemented
    static speed_t getBaudRateConstant(int baudRate)
    {
        switch(baudRate)
        {
#define BAUD_RATE(v) case v: return B ## v
            BAUD_RATE(50);
            BAUD_RATE(75);
            BAUD_RATE(110);
            BAUD_RATE(134);
            BAUD_RATE(150);
            BAUD_RATE(200);
            BAUD_RATE(300);
            BAUD_RATE(600);
            BAUD_RATE(1200);
            BAUD_RATE(1800);
            BAUD_RATE(2400);
            BAUD_RATE(4800);
            BAUD_RATE(9600);
            BAUD_RATE(19200);
            BAUD_RATE(38400);
#ifdef B57600
            BAUD_RATE(57600);
#endif
#ifdef B115200
            BAUD_RATE(115200);
#endif
#ifdef B230400
            BAUD_RATE(230400);
#endif
#ifdef B460800
            BAUD_RATE(460800);
#endif
#ifdef B500000
            BAUD_RATE(500000);
#endif
#ifdef B576000
            BAUD_RATE(576000);
#endif
#ifdef B921600
            BAUD_RATE(921600);
#endif
#ifdef B1000000
            BAUD_RATE(1000000);
#endif
#ifdef B1152000
            BAUD_RATE(1152000);
#endif
#ifdef B1500000
            BAUD_RATE(1500000);
#endif
#ifdef B2000000
            BAUD_RATE(2000000);
#endif
#ifdef B2500000
            BAUD_RATE(2500000);
#endif
#ifdef B3000000
            BAUD_RATE(3000000);
#endif
#ifdef B3500000
            BAUD_RATE(3500000);
#endif
#ifdef B4000000
            BAUD_RATE(4000000);
#endif
#undef BAUD_RATE

        default:
            return B0;
        }
    }
public:
    SerialOutput(string devicePath, int baudRate)
    {
        if(baudRate < 10)
            error("invalid baud rate");
        signal(SIGIO, SIG_IGN);
        int fd = open(devicePath.c_str(), O_NOCTTY | O_RDWR);

        if(fd == -1)
        {
            error("can't open : " + devicePath);
        }

        termios settings;

        if(0 != tcgetattr(fd, &settings))
        {
            error("can't get settings : " + devicePath);
        }

        cfmakeraw(&settings);
        speed_t baudRateConstant = getBaudRateConstant(baudRate);

        if(baudRateConstant == B0) // set custom divisor
        {
            serial_struct serial;
            serial.reserved_char[0] = 0;
            if(ioctl(fd, TIOCGSERIAL, &serial) < 0)
                error("can't get settings : " + devicePath);
            serial.flags &= ~ASYNC_SPD_MASK;
            serial.flags |= ASYNC_SPD_CUST;
            serial.custom_divisor = (serial.baud_base + baudRate / 2) / baudRate;
            if(serial.custom_divisor < 1)
                serial.custom_divisor = 1;
            if(ioctl(fd, TIOCSSERIAL, &serial) < 0)
                error("can't set settings : " + devicePath);
        }

        cfsetispeed(&settings, baudRateConstant);
        cfsetospeed(&settings, baudRateConstant);
        settings.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
        settings.c_cflag |= CLOCAL | CREAD | CS8;

        if(0 != tcsetattr(fd, TCSANOW, &settings))
        {
            error("can't set settings : " + devicePath);
        }

        tcflush(fd, TCIOFLUSH);
        f = fdopen(fd, "wb");

        if(!f)
        {
            error("can't run fdopen");
        }

        vector<uint8_t> buffer;
        // write a data chunk time it to derive actual baud rate
        buffer.resize(1024, 0xAA);
        int iterations = 1;

        for(;;)
        {
            timespec startTime, endTime;
            clock_gettime(CLOCK_MONOTONIC, &startTime);
            size_t totalSize = 0;

            for(int i = 0; i < iterations; i++)
            {
                fwrite((const void *)&buffer[0], sizeof(buffer[0]), buffer.size(), f);
                totalSize += buffer.size();
            }

            tcdrain(fd);
            clock_gettime(CLOCK_MONOTONIC, &endTime);
            double fStartTime = startTime.tv_sec + startTime.tv_nsec * 1e-9;
            double fEndTime = endTime.tv_sec + endTime.tv_nsec * 1e-9;
            double elapsedTime = fEndTime - fStartTime;
            double totalBits = (double)totalSize * (1/* start bit */ + 8/* data bits */ + 1/* stop bit */);

            if(elapsedTime < 1)
            {
                iterations *= 2;
                continue;
            }

            actualBaudRate = totalBits / elapsedTime;
            break;
        }
    }
    void write(uint8_t v)
    {
        fputc(v, f);
    }
    ~SerialOutput()
    {
        fclose(f);
    }
    float getActualBaudRate() const
    {
        return actualBaudRate;
    }
};

class SignalSource
{
    SignalSource(const SignalSource &); // not implemented
    void operator =(const SignalSource &); // not implemented
public:
    bool done;
    SignalSource()
        : done(false)
    {
    }
    virtual ~SignalSource()
    {
    }
    virtual float operator ()(float deltaTime) = 0;
};

class SignalSourceFile : public SignalSource
{
    double sampleRate;
    int bytesPerSample;
    ifstream f;
    float lastSample, currentSample;
    float fractSample;
    bool isSigned, isLittleEndian;
    int channelCount;
    float readSampleInternal()
    {
        if(done)
        {
            return 0;
        }

        int b = f.get();

        if(b == EOF)
        {
            done = true;
            return 0;
        }

        int32_t minValue = -0x80 << 8 * (bytesPerSample - 1);
        int32_t value = b;
        if(!isLittleEndian)
            value <<= 8 * (bytesPerSample - 1);
        int32_t mask = -0x100 << 8 * (bytesPerSample - 1);

        for(int i = 1; i < bytesPerSample; i++)
        {
            b = f.get();

            if(b == EOF)
            {
                done = true;
                return 0;
            }

            if(isLittleEndian)
                value |= (int32_t)b << 8 * i;
            else
                value |= (int32_t)b << 8 * (bytesPerSample - i - 1);
        }

        if(!isSigned)
        {
            return (float)(int64_t)(uint32_t)value / -(float)minValue - 1;
        }

        if(value & minValue)
        {
            value |= mask;
        }

        return -(float)value / minValue;
    }
    float readSample()
    {
        float retval = 0;
        for(int i = 0; i < channelCount; i++)
            retval += readSampleInternal();
        return retval / channelCount;
    }
public:
    SignalSourceFile(string fileName, double sampleRate, int bytesPerSample, bool isSigned, bool isLittleEndian, int channelCount)
        : sampleRate(sampleRate), bytesPerSample(bytesPerSample), fractSample(0), isSigned(isSigned), isLittleEndian(isLittleEndian), channelCount(channelCount)
    {
        f.open(fileName.c_str(), ios::binary | ios::in);

        if(!f)
        {
            error("can't open file : " + fileName);
        }

        lastSample = readSample();
        currentSample = readSample();
    }
    ~SignalSourceFile()
    {
    }
    float operator()(float deltaTime)
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
#ifdef USE_VORBISFILE
class SignalSourceVorbisFile : public SignalSource
{
    OggVorbis_File ovf;
    FILE *f;
    unsigned channels;
    unsigned sampleRate;
    vector<float> buffer;
    size_t currentBufferPos;
    float fractSample;
    float lastSample, currentSample;
    static size_t read_fn(void *dataPtr_in, size_t blockSize, size_t numBlocks, void *dataSource)
    {
        SignalSourceVorbisFile &decoder = *(SignalSourceVorbisFile *)dataSource;
        return fread(dataPtr_in, blockSize, numBlocks, decoder.f);
    }
    inline void readBuffer()
    {
        buffer.resize(buffer.capacity());
        int currentSection;
        currentBufferPos = 0;
        float **pcmChannels = NULL;
        long sampleCountLong = ov_read_float(&ovf, &pcmChannels, buffer.size() / channels, &currentSection);

        if(sampleCountLong < 0)
        {
            sampleCountLong = 0;
        }

        size_t sampleCount = sampleCountLong;
        buffer.resize(sampleCount * channels);

        for(size_t sample = 0, bufferIndex = 0; sample < sampleCount; sample++)
        {
            for(unsigned channel = 0; channel < channels; channel++)
            {
                buffer[bufferIndex++] = pcmChannels[channel][sample];
            }
        }
    }
    float readSample()
    {
        if(currentBufferPos >= buffer.size())
        {
            readBuffer();

            if(buffer.size() == 0)
            {
                done = true;
                return 0;
            }
        }

        float retval = 0;
        for(unsigned j = 0; j < channels; j++)
        {
            retval += buffer[currentBufferPos++];
        }
        retval /= channels;
        return retval;
    }
public:
    SignalSourceVorbisFile(string fileName)
        : currentBufferPos(0), fractSample(0)
    {
        f = fopen(fileName.c_str(), "rb");

        if(!f)
        {
            error("can't open file : " + fileName);
        }

        ov_callbacks callbacks;
        callbacks.read_func = &read_fn;
        callbacks.seek_func = NULL;
        callbacks.close_func = NULL;
        callbacks.tell_func = NULL;
        int openRetval = ov_open_callbacks((void *)this, &ovf, NULL, 0, callbacks);

        switch(openRetval)
        {
        case 0:
            break;

        default:
            error("invalid ogg vorbis audio");
        }

        vorbis_info *info = ov_info(&ovf, -1);
        channels = info->channels;
        buffer.reserve(channels * 8192);
        sampleRate = info->rate;

        if(channels == 0 || sampleRate == 0)
        {
            ov_clear(&ovf);
            error("invalid ogg vorbis audio");
        }

        lastSample = readSample();
        currentSample = readSample();
    }
    ~SignalSourceVorbisFile()
    {
        ov_clear(&ovf);
        fclose(f);
    }
    float operator()(float deltaTime)
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
#endif
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
    float operator()(float deltaTime)
    {
        angle += 2 * M_PI * frequency * deltaTime;
        angle = fmod(angle, 2 * M_PI);
        return amplitude * sin(angle);
    }
};
#if 1
int main(int argc, char **argv)
{
#ifdef USE_VORBISFILE
    string signalSourceFile = "test.ogg";
    bool isOgg = true;
#else
    string signalSourceFile = "audio-data-16le-44100-mono.bin";
#endif
    string outputDevice = "/dev/ttyUSB0";
    int signalSourceSampleRate = 44100;
    int signalSourceBytesPerSample = 2;
    bool signalSourceIsSigned = true;
    bool signalSourceIsLittleEndian = true;
    int signalSourceChannelCount = 1;
    int outputBaudRate = 1000000;
    int optchar;
    opterr = 0;
    float amplitude = 1.0f;

    while(-1 != (optchar = getopt(argc, argv, "hd:i:r:z:b:sua:lB")))
    {
        switch(optchar)
        {
        case 'h':
            cout << "delta-sigma-test v0.1 by Jacob Lifshay (c) 2014\n"
                 "options:\n"
                 "general:\n"
                 "-h\t\tshow this help.\n"
                 "output:\n"
                 "-a <number>\tset output amplitude.\n"
                 "-d <path>\tset serial device.\n"
                 "-b <rate>\tset output baud rate.\n"
                 "input:\n"
                 "-i <file>\tset input file.\n"
                 "raw input:\n"
                 "-r <rate>\tset input sample rate.\n"
                 "-z <size>\tset the sample size in bytes.\n"
                 "-c <count>\tset the channel count.\n"
                 "-s\t\tset input sample type to signed.\n"
                 "-u\t\tset input sample type to unsigned.\n"
                 "-l\t\tset input sample type to little endian.\n"
                 "-B\t\tset input sample type to big endian.\n"
                 ;
            cout << flush;
            return 0;

        case 'a':
            if(1 != sscanf(optarg, " %f", &amplitude) || !isfinite(amplitude) || amplitude <= 0 || amplitude > 1000)
            {
                error((string)"invalid amplitude : " + optarg);
            }

            break;

        case 'd':
            outputDevice = optarg;
            break;

        case 'i':
            signalSourceFile = optarg;
            break;
        case 'r':
#ifdef USE_VORBISFILE
            isOgg = false;
#endif
            if(1 != sscanf(optarg, " %i", &signalSourceSampleRate) || signalSourceSampleRate < 100 || signalSourceSampleRate > 1000000)
            {
                error((string)"invalid sample rate : " + optarg);
            }

            break;

        case 'z':
#ifdef USE_VORBISFILE
            isOgg = false;
#endif
            if(1 != sscanf(optarg, " %i", &signalSourceBytesPerSample) || signalSourceBytesPerSample < 1 || signalSourceBytesPerSample > 4)
            {
                error((string)"invalid sample size : " + optarg);
            }

            break;
        case 'c':
#ifdef USE_VORBISFILE
            isOgg = false;
#endif
            if(1 != sscanf(optarg, " %i", &signalSourceChannelCount) || signalSourceChannelCount < 1 || signalSourceChannelCount > 100)
            {
                error((string)"invalid channel count : " + optarg);
            }

            break;
        case 'b':
            if(1 != sscanf(optarg, " %i", &outputBaudRate) || outputBaudRate < 50 || outputBaudRate > 1000000000)
            {
                error((string)"invalid baud rate : " + optarg);
            }

            break;
        case 's':
#ifdef USE_VORBISFILE
            isOgg = false;
#endif
            signalSourceIsSigned = true;
            break;

        case 'u':
#ifdef USE_VORBISFILE
            isOgg = false;
#endif
            signalSourceIsSigned = false;
            break;
        case 'l':
#ifdef USE_VORBISFILE
            isOgg = false;
#endif
            signalSourceIsLittleEndian = true;
            break;

        case 'B':
#ifdef USE_VORBISFILE
            isOgg = false;
#endif
            signalSourceIsLittleEndian = false;
            break;
        case '?':
            if(optopt == 'd' || optopt == 'i' ||
               optopt == 'r' || optopt == 'z' ||
               optopt == 'b' || optopt == 'a' ||
               optopt == 'c')
            {
                error((string)"-" + (char)optopt + " missing argument");
            }

            error((string)"invalid option : -" + (char)optopt);
            break;

        default:
            error((string)"invalid option : " + (char)optchar);
            break;
        }
    }

    if(optind < argc)
    {
        error((string)"unexpected argument : " + argv[optind]);
    }

    SigmaDeltaModulator sdm;
    SerialOutput serialOutput(outputDevice, outputBaudRate);
    SignalSource *pSignalSource = NULL;
#ifdef USE_VORBISFILE
    if(isOgg)
        pSignalSource = new SignalSourceVorbisFile(signalSourceFile);
#endif
    if(!pSignalSource)
        pSignalSource = new SignalSourceFile(signalSourceFile, signalSourceSampleRate, signalSourceBytesPerSample, signalSourceIsSigned, signalSourceIsLittleEndian, signalSourceChannelCount);
    SignalSource &signalSource = *pSignalSource;
    int bitNumber = 0;
    uint8_t byteValue = 0;
    size_t currentBitCount = 0;
    cout << "actual baud rate : " << fixed << serialOutput.getActualBaudRate() << endl;
    float bitPeriod = 1.0f / serialOutput.getActualBaudRate();
    double elapsedTime = 0, lastMessageTime = -1e8;

    while(!signalSource.done)
    {
        if(elapsedTime - lastMessageTime >= 0.1)
        {
            lastMessageTime = elapsedTime;
            double seconds = elapsedTime;
            double minutes = std::floor(seconds / 60);
            double hours = std::floor(minutes / 60);
            double days = std::floor(hours / 24);
            seconds -= minutes * 60;
            minutes -= hours * 60;
            hours -= days * 24;
            long long idays = (long long)days;
            int ihours = (int)std::floor(hours + 0.5);
            int iminutes = (int)std::floor(minutes + 0.5);
            char str[256];
            snprintf(str, sizeof(str), "%llid %i:%02i:%02.3f", idays, ihours, iminutes, seconds);
            cout << "\r" << str << "\x1b[K\r" << flush;
        }
        float signal = signalSource(bitPeriod) * amplitude;
        elapsedTime += bitPeriod;
        bool sdmOutput;
        const int startBitCount = 1, stopBitCount = 1, dataBitCount = 8;
        signal *= dataBitCount / (double)(startBitCount + stopBitCount + dataBitCount);
        bitNumber++;
        bitNumber %= startBitCount + stopBitCount + dataBitCount;

        if(bitNumber < startBitCount)
        {
            sdmOutput = sdm.forcedStep(signal, false);
        }
        else if(bitNumber < startBitCount + dataBitCount)
        {
            sdmOutput = sdm.step(signal);
            byteValue >>= 1;

            if(sdmOutput)
            {
                byteValue |= 0x80;
            }

            if(++currentBitCount >= dataBitCount)
            {
                currentBitCount = 0;
                serialOutput.write(byteValue >> (8 - dataBitCount));
            }
        }
        else
        {
            sdmOutput = sdm.forcedStep(signal, true);
        }
    }
    delete pSignalSource;

    return 0;
}
#else
int main(int argc, char **argv)
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
        {
            sdm.step(signal);
        }

        for(int y = 0; y < 20; y++)
        {
            for(int x = 0; x < 70; x++)
            {
                bool v;

                if(x % 10 < 2)
                {
                    v = sdm.forcedStep(signal, x % 10 == 0);
                }
                else
                {
                    v = sdm.step(signal);
                }

                if(v)
                {
                    cout << "#";
                }
                else
                {
                    cout << " ";
                }
            }

            cout << "\x1b[K\n";
        }

        cout << flush;
    }

    return 0;
}
#endif
