/*
 * Copyright (C) 2026 Honu Robotics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

// Unit tests for the column-major -> row-major grid reflow used by
// HeightMapTexture when forwarding a WaveField2D to the Ogre2 bridge. The
// helper is the only data transform on the upload path and has no GUI/Ogre
// dependency, so it is tested here in isolation.

#include <cstddef>
#include <vector>

#include <gtest/gtest.h>

#include "GridReflow.hh"

namespace gss = gz::sim::systems;

//////////////////////////////////////////////////
// The contract: out[i*N + j] == in[i + j*N] for every (i, j).
TEST(GridReflow, TransposesColumnMajorToRowMajor)
{
  constexpr std::size_t N = 4;
  // Column-major source: element (i, j) at index i + j*N, encoded as 10*i + j
  // so each cell's (i, j) is recoverable.
  std::vector<double> col(N * N);
  for (std::size_t j = 0; j < N; ++j)
    for (std::size_t i = 0; i < N; ++i)
      col[i + j * N] = 10.0 * i + j;

  const std::vector<double> row = gss::ToRowMajor(col.data(), N);
  ASSERT_EQ(row.size(), N * N);
  for (std::size_t i = 0; i < N; ++i)
    for (std::size_t j = 0; j < N; ++j)
      EXPECT_DOUBLE_EQ(row[i * N + j], 10.0 * i + j)
        << "mismatch at (i=" << i << ", j=" << j << ")";
}

//////////////////////////////////////////////////
// A null channel (absent dx/dy/foam) reflows to all zeros.
TEST(GridReflow, NullChannelYieldsZeros)
{
  const std::vector<double> row = gss::ToRowMajor(nullptr, 5);
  ASSERT_EQ(row.size(), 25u);
  for (const double v : row)
    EXPECT_DOUBLE_EQ(v, 0.0);
}

//////////////////////////////////////////////////
// Degenerate sizes are handled without crashing.
TEST(GridReflow, ZeroSizeIsEmpty)
{
  EXPECT_TRUE(gss::ToRowMajor(nullptr, 0).empty());
  const double one = 42.0;
  const std::vector<double> row = gss::ToRowMajor(&one, 1);
  ASSERT_EQ(row.size(), 1u);
  EXPECT_DOUBLE_EQ(row[0], 42.0);
}
