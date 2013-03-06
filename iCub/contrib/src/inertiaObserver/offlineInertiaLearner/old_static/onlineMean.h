/**
 * A class for online mean calculation
 */
template <class T>
class onlineMean {
    T current_mean;
    unsigned int sample_num;
    
    public:
    onlineMean();
    ~onlineMean() {};
    void feedSample(const T& sample);
    void reset();
    T getMean();
    unsigned int getSampleNum();
};


template<class T>
onlineMean<T>::onlineMean() {
    this->reset();
}

template<class T>
void onlineMean<T>::feedSample(const T& sample) {
    T delta;
    sample_num++;
    if( sample_num == 1 ) {
        current_mean = sample;
    } else {
        delta = sample-current_mean;
        current_mean += delta/sample_num;
    }
    return;
}

template<class T>
void onlineMean<T>::reset() {
    sample_num = 0;
}

template<class T>
T onlineMean<T>::getMean() {
    return current_mean;
}

template<class T>
unsigned int  onlineMean<T>::getSampleNum() {
    return sample_num;
}

