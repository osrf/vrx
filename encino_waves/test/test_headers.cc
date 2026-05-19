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
 * Compile-only test: include every vendored EncinoWaves header so the
 * compiler tells us about leftover FFTW / Alembic references after the
 * Foundation.h surgery. If this links it doesn't mean the library WORKS,
 * only that the headers + the Eigen-backed FftwWrapper agree on the type
 * surface.
 */

#include "EncinoWaves/All.h"

int main()
{
  return 0;
}
