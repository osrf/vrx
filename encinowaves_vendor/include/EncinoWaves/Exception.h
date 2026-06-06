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

//-*****************************************************************************
// The basic architecture of these Waves is based on the TweakWaves application
// written by Chris Horvath for Tweak Films in 2001.  This, in turn, was based
// on the SIGGRAPH papers and courses by Jerry Tessendorf, and by the paper
// "A Simple Fluid Solver based on the FTT" by Jos Stam.
//
// The TMA, JONSWAP, and Pierson Moskowitz Wave Spectra, as well as the
// directional spreading functions are formulated based on the descriptions
// given in "Ocean Waves: The Stochastic Approach",
// by Michel K. Ochi, published by Cambridge Ocean Technology Series, 1998,2005.
//
// This library is written as a working implementation of the paper:
// Christopher J. Horvath. 2015.
// Empirical directional wave spectra for computer graphics.
// In Proceedings of the 2015 Symposium on Digital Production (DigiPro '15),
// Los Angeles, Aug. 8, 2015, pp. 29-39.
//-*****************************************************************************

#ifndef _EncinoWaves_Exception_h_
#define _EncinoWaves_Exception_h_

#include "Foundation.h"

namespace EncinoWaves {

//-*****************************************************************************
//! Base class for all exceptions in the Emerald libraries. Derived
//! from both std::exception and std::string, publicly
//! It is mostly commonly thrown using the macros
class Exception : public std::string, public std::exception {
public:
  //! default constructor creates exception with
  //! empty message string
  Exception() throw()
      : std::string("")
      , std::exception() {}

  //!  Creates exception with an explicit message string.
  //! ...
  explicit Exception(const std::string &str) throw()
      : std::string(str)
      , std::exception() {}

  //!  Copies exception.
  //! ...
  Exception(const Exception &exc) throw()
      : std::string(exc.c_str())
      , std::exception() {}

  //!  Destructor is empty, but virtual to support polymorphic
  //! destruction of data in any derived classes.
  virtual ~Exception() throw() {}

  //!  Inherited from std::exception, this returns a non-modifiable
  //! character string describing the nature of the exception
  virtual const char *what() const throw() { return c_str(); }
};

//-*****************************************************************************
// This is for aborting in a debug mode to make a more traceable stack.

#ifdef PLATFORM_DARWIN

#if defined __cplusplus
#define __EWAV_DEBUG_ASSERT_VOID_CAST static_cast<void>
#else
#define __EWAV_DEBUG_ASSERT_VOID_CAST (void)
#endif

#else

#if defined __cplusplus && __GNUC_PREREQ(2, 95)
#define __EWAV_DEBUG_ASSERT_VOID_CAST static_cast<void>
#else
#define __EWAV_DEBUG_ASSERT_VOID_CAST (void)
#endif

#endif

extern void __EWAV_DEBUG_ASSERT_FAIL(const char *msg) throw();

//-*****************************************************************************
// This macro will cause an abort.
#define EWAV_FAIL(TEXT)                                       \
  do {                                                        \
    std::stringstream sstr;                                   \
    sstr << TEXT;                                             \
    sstr << "\nFile: " << __FILE__ << std::endl               \
         << "Line: " << __LINE__ << std::endl;                \
    EncinoWaves::__EWAV_DEBUG_ASSERT_FAIL(sstr.str().c_str()); \
  } while (0)

//-*****************************************************************************
//! convenient macro which may be used with std::iostream syntax
//! EWAV_THROW( "this integer: " << myInt << " is bad" )
//#ifdef DEBUG
#if 0

#define EWAV_THROW(TEXT)            \
  do {                              \
    std::cerr << TEXT << std::endl; \
    abort();                        \
  } while (0)

#else

#define EWAV_THROW(TEXT)                        \
  do {                                          \
    std::stringstream sstr;                     \
    sstr << TEXT;                               \
    sstr << "\nFile: " << __FILE__ << std::endl \
         << "Line: " << __LINE__ << std::endl;  \
    EncinoWaves::Exception exc(sstr.str());      \
    throw(exc);                                 \
  } while (0)

#endif

//-*****************************************************************************
#define EWAV_ASSERT(COND, TEXT)              \
  do {                                            \
    if (!(COND)) {                                \
      std::stringstream sstr;                     \
      sstr << TEXT;                               \
      sstr << "\nFile: " << __FILE__ << std::endl \
           << "Line: " << __LINE__ << std::endl;  \
      EncinoWaves::Exception exc(sstr.str());      \
      throw(exc);                                 \
    }                                             \
  } while (0)

//-*****************************************************************************
#ifdef DEBUG

#define EWAV_ASSERT(COND, TEXT) \
  do {                          \
    if (!(COND)) {              \
      EWAV_FAIL(TEXT);          \
    }                           \
  } while (0)

#define EWAV_DEBUG_ASSERT(COND, TEXT) \
  do {                                \
    if (!(COND)) {                    \
      EWAV_FAIL(TEXT);                \
    }                                 \
  } while (0)

#else

#define EWAV_ASSERT(COND, TEXT) \
  do {                          \
    if (!(COND)) {              \
      EWAV_THROW(TEXT);         \
    }                           \
  } while (0)

#define EWAV_DEBUG_ASSERT(COND, TEXT) (__EWAV_DEBUG_ASSERT_VOID_CAST(0))

#endif

}  // namespace EncinoWaves

#endif
