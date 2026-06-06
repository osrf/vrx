/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 *
 * Static singletons for the no-op thread-init helpers. Matches the layout
 * of upstream EncinoWaves's FftwWrapper.cpp so any translation unit that
 * still references `__BaseFftwInitThreadsT<T>::sm_init` keeps linking.
 */

#include "EncinoWaves/FftwWrapper.h"

namespace EncinoWaves
{

std::unique_ptr<__BaseFftwInitThreadsT<float>::Init>
    __BaseFftwInitThreadsT<float>::sm_init;

std::unique_ptr<__BaseFftwInitThreadsT<double>::Init>
    __BaseFftwInitThreadsT<double>::sm_init;

}  // namespace EncinoWaves
