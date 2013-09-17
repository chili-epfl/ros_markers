#ifndef FILTER_H
#define FILTER_H

#define NUM_ELTS 20 // max num of elements for filtering

template < typename T >
class CircularVector
{
public:
    CircularVector() : idx(0)
    {
        data = vector<T>(NUM_ELTS);
    }
    void push_back(T& elt)
    {
        data[ idx++ % NUM_ELTS ] = elt;
    }

    vector<T> data;

private:
    int idx;
};


class Filter {

public:
    CircularVector<float> vals;

    void add(float x);
    float mean();
    float variance();
    float stddev();
};



#endif
