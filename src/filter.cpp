#include "filter.h"


void Filter::add(float x)
{
    vals.push_back(x);
}

float Filter::mean()
{
    float sum = 0.0;
    for(float a : vals.data) sum += a;
    return sum/NUM_ELTS;
}

float Filter::variance()
{
    float current_mean = mean();
    float temp = 0;
    for(float a : vals.data)
        temp += (current_mean-a)*(current_mean-a);
    return temp/NUM_ELTS;
}

float Filter::stddev()
{
    return sqrt(variance());
}


