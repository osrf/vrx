/*
 * Copyright 2015 Christopher Jon Horvath
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *
 * Stripped-down replacement for upstream `Util/Foundation.h`.
 *
 * The original pulls in Alembic (`Alembic/AbcGeom/All.h`) purely to import a
 * pile of typedefs (V2f, V2d, Box3f, M44f, half-precision types, ...) and
 * the OpenEXR `half.h` library. None of those names are actually referenced
 * by the EncinoWaves wave code we want to use — a grep across the
 * EncinoWaves headers shows it only ever names `Imath::V2i`,
 * `Imath::Vec2<T>`, `Imath::Vec3<T>`, `Imath::Box<>`, `Imath::clamp`, and
 * `Imath::lerp`. So dropping Alembic + half costs us nothing while shedding
 * two non-trivial third-party deps.
 *
 * What remains here is just the standard-library kitchen sink that the rest
 * of EncinoWaves expects to be in scope after including Foundation.h, plus a
 * couple of size-typedefs we promote from <cstdint> in case anything reaches
 * for them.
 */

#ifndef _EncinoWaves_Util_Foundation_h_
#define _EncinoWaves_Util_Foundation_h_

#include <sys/time.h>

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

#include <ImathBox.h>
#include <ImathFun.h>
#include <ImathMath.h>
#include <ImathVec.h>

namespace EncinoWaves
{
namespace Util
{

// Standard sized integers — re-exported in the namespace so EncinoWaves code
// can keep using the bare names (it never does today, but matches the
// upstream surface area in case Propagation.h or a future addition reaches
// for them).
using std::int8_t;
using std::uint8_t;
using std::int16_t;
using std::uint16_t;
using std::int32_t;
using std::uint32_t;
using std::int64_t;
using std::uint64_t;

}  // namespace Util
}  // namespace EncinoWaves

#endif  // _EncinoWaves_Util_Foundation_h_
