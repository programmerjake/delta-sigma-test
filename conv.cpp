#include <iostream>
#include <string>

int main()
{
    static char bitCountTable[256] = {0};
    static unsigned char translateTable[256];
    for(int i = 0; i < 256; i++)
    {
        char bitCount = 0;
        if(i > 0)
        {
            bitCount = 1 + bitCountTable[i & (i - 1)];
        }
        bitCountTable[i] = bitCount;
        int v = bitCount;
        v = (v + 1) * 255 / 10;
        translateTable[i] = v;
    }
    int currentValue = 0, valueCount = 0;
    for(int ch = std::cin.get(); std::char_traits<char>::not_eof(ch) == ch; ch = std::cin.get())
    {
        int v = translateTable[ch & 0xFF];
        currentValue += v;
        valueCount++;
        valueCount %= 10;
        if(valueCount == 0)
        {
	    std::cout.put(currentValue / 10);
            currentValue = 0;
        }
    }
    return 0;
}
