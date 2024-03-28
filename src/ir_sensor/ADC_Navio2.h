#pragma once

#include <cstddef>
//#include <Common/ADC.h>

class ADC_Navio2 
{
public:
    void initialize();
    int get_channel_count(void) ;
    int read(int ch);
    ADC_Navio2();
    ~ADC_Navio2();

private:
    int open_channel(int ch);

    static const size_t CHANNEL_COUNT = 6;
    int channels[CHANNEL_COUNT];
};
