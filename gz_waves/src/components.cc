/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

/// \file
/// \brief Anchors the `GZ_SIM_REGISTER_COMPONENT` static initializer for
/// `components::Wavefield` in the core `libwaves.so`. Every system plugin we
/// build links against this library, so the registration runs as soon as any
/// of the plugins is dlopen'd — including in the GUI process, where the
/// component needs to be registered before SceneBroadcaster's serialized
/// state arrives.

#include "gz/sim/components/Wavefield.hh"
