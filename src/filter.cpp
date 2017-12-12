#include "filter.h"

acc_filter::acc_filter(float cutoff, float samplefreq, int type):
 type_(type) {
  params_.resize(1);
  nfbuf_.resize(2);
  fbuf_.resize(2);
  float RC = 1.0 / (2.0 * 3.14 * cutoff);
  params_[0] = RC / (RC + 1.0 / samplefreq);
};
