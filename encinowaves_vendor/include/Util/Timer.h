//-*****************************************************************************
// Copyright 2015 Christopher Jon Horvath
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//-*****************************************************************************

#ifndef _EncinoWaves_Util_Timer_h_
#define _EncinoWaves_Util_Timer_h_

#include "Foundation.h"

namespace EncinoWaves {
namespace Util {

//-*****************************************************************************
//! \brief Basic real-time stopwatch. Returns elapsed time in seconds.
class Timer {
public:
  //! Begins the timer and resets the elapsed time to zero
  void start() {
    struct timezone tz;
    gettimeofday(&m_tp0, &tz);
    m_stopped = -1;
  }

  //! Creates a timer which is started by default
  Timer()
    : m_stopped(-1) {
    start();
  }

  //! Stops the timer and records elapsed time
  double stop() {
    struct timezone tz;
    struct timeval tp;
    gettimeofday(&tp, &tz);

    double seconds = tp.tv_sec - m_tp0.tv_sec;
    seconds += double(tp.tv_usec) * 1e-6;
    seconds -= double(m_tp0.tv_usec) * 1e-6;
    return (m_stopped = double(seconds));
  }

  //! Returns the amount of time elapsed.
  double elapsed() const {
    if (m_stopped >= 0.0) {
      return m_stopped;
    }

    struct timezone tz;
    struct timeval tp;
    gettimeofday(&tp, &tz);

    double seconds = tp.tv_sec - m_tp0.tv_sec;
    seconds += double(tp.tv_usec) * 1e-6;
    seconds -= double(m_tp0.tv_usec) * 1e-6;
    return double(seconds);
  }

private:
  double m_stopped;
  struct timeval m_tp0;
};

}  // namespace Util
}  // namespace EncinoWaves

#endif
