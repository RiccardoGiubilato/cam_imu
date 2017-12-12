#ifndef filter
#define filter

#include <vector>
#include <iostream>

#define HIGHPASS 1
#define LOWPASS 2

class acc_filter {

 public:
  acc_filter();
  acc_filter(float cutoff, float samplefreq, int type);
  ~acc_filter(){};

  /* update buffer and process new data */
  template<typename T>
  void process(T& new_);

 private:
  std::vector<double> nfbuf_, fbuf_;
  std::vector<double> params_;
  int type_;

};

template<typename T>
void acc_filter::process(T& new_) {
  nfbuf_[0] = nfbuf_[1];
  nfbuf_[1] = new_;
  fbuf_[0] = fbuf_[1];
  if (fbuf_[1] != 0) {
    if (type_ == HIGHPASS) new_ = params_[0] * (fbuf_[0] + nfbuf_[1] - nfbuf_[0]);
    if (type_ == LOWPASS) new_ = 0.25 * new_ + (1 - 0.25) * fbuf_[0];
    fbuf_[1] = new_;
  } else {
    fbuf_[1] = new_;
  }
};

#endif
