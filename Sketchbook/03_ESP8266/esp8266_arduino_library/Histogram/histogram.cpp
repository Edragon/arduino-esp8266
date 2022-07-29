//
//
//    FILE: Histogram.h
//  AUTHOR: Rob Tillaart
// VERSION: 0.1.04
// PURPOSE: Histogram library for Arduino
//    DATE: 2012-11-10
//
// Released to the public domain
//
// HISTORY:
// 0.1.0 - 2012-11-10 initial version
// 0.1.1 - 2012-11-10 added PMF() and CDF()
// 0.1.2 - 2012-12-23 changed float to double; some comments
// 0.1.03 - 2013-09-29 testing a lot & refactoring
// 0.1.04 - 2015-03-06 stricter interface
//
// Released to the public domain
//

#include "histogram.h"

Histogram::Histogram(const uint8_t len, double *bounds)
{
    _bounds = bounds;
    _len = len + 1;
    _data = (long*) malloc((_len) * sizeof(long));
    clear();
}

Histogram::~Histogram()
{
    free(_data);  // free may still have a bug :(
}

// resets all counters
void Histogram::clear()
{
    for (uint8_t i = 0; i < _len; i++)
    {
        _data[i] = 0;
    }
    _cnt = 0;
}

// adds a new value to the histogram - increasing
void Histogram::add(const double f)
{
    _data[find(f)]++;
    _cnt++;
}

// adds a new value to the histogram - decreasing
void Histogram::sub(const double f)
{
    _data[find(f)]--;
    _cnt++;;
}

// returns the number of buckets
uint8_t Histogram::size()
{
    return _len;
}

// returns the number of values added
unsigned long Histogram::count()
{
    return _cnt;
}

// returns the count of a bucket
long Histogram::bucket(const uint8_t idx)
{
    if (idx > _len) return 0;
    return _data[idx];
}

// returns the relative frequency of a bucket
double Histogram::frequency(const uint8_t idx)
{
    if (_cnt == 0) return NAN;
    if (idx > _len) return 0;   // diff with PMF
    return (1.0 * _data[idx]) / _cnt;
}

// EXPERIMENTAL
// returns the probability of the bucket of a value
double Histogram::PMF(const double val)
{
    if (_cnt == 0) return NAN;
    uint8_t idx = find(val);
    return (1.0 *_data[idx]) / _cnt;
}

// EXPERIMENTAL
// returns the cummulative probability of
// values <= value
double Histogram::CDF(const double val)
{
    if (_cnt == 0) return NAN;
    uint8_t idx = find(val);
    long sum = 0;
    for (uint8_t i=0; i<= idx; i++)
    {
        sum += _data[i];
    }
    return (1.0 * sum) / _cnt;
}

// EXPERIMENTAL
// returns the value of the original array for
// which the CDF is at least prob.
double Histogram::VAL(const double prob)
{
    if (_cnt == 0) return NAN;
    double p = prob;
    if (p < 0.0) p = 0.0;
    if (p > 1.0) p = 1.0;

    double value = p * _cnt;
    long sum = 0;
    for (uint8_t i = 0; i < _len; i++)
    {
        sum += _data[i];
        if (sum >= value && (i <(_len-1)) ) return _bounds[i];
    }
    return INFINITY;
}

// returns the bucket number for value val
uint8_t Histogram::find(const double val)
{
    for (uint8_t i = 0; i< (_len-1); i++)
    {
        if (_bounds[i] >= val) return i;
    }
    return _len-1;
    // uint8_t i = 0;
    // while ((i < (_len-1)) && (_bounds[i] < val)) i++;
    // return i;
}

// END OF FILE