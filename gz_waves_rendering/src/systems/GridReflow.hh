/*
 * Copyright (C) 2026 Honu Robotics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

#ifndef GZ_SIM_SYSTEMS_GRIDREFLOW_HH_
#define GZ_SIM_SYSTEMS_GRIDREFLOW_HH_

#include <cstddef>
#include <vector>

// Pure, dependency-free grid helper used by HeightMapTexture. Kept out of the
// Ogre-laden HeightMapTexture header so it can be unit-tested on its own.
namespace gz::sim::systems
{
  /// \brief Reflow a column-major `_n × _n` grid (the `WaveField2D` layout,
  /// element (i, j) at index `i + j*_n`) into a row-major copy (element (i, j)
  /// at `i*_n + j`), which is what the Ogre2 heightmap bridge's C-ABI expects.
  /// A null input yields an all-zero grid (used for absent displacement/foam
  /// channels).
  /// \param[in] _col Source column-major buffer, or null for zeros.
  /// \param[in] _n   Grid resolution per axis.
  /// \return The row-major copy (`_n * _n` elements).
  inline std::vector<double> ToRowMajor(const double *_col, std::size_t _n)
  {
    std::vector<double> row(_n * _n, 0.0);
    if (_col)
    {
      for (std::size_t j = 0; j < _n; ++j)
        for (std::size_t i = 0; i < _n; ++i)
          row[i * _n + j] = _col[i + j * _n];
    }
    return row;
  }
}  // namespace gz::sim::systems

#endif  // GZ_SIM_SYSTEMS_GRIDREFLOW_HH_
