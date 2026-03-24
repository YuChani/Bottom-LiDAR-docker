// SPDX-License-Identifier: MIT
// Copyright (c) 2026

#include <gtsam_points/types/point_cloud.hpp>
#include "ndt/integrated_light_ndt_factor.hpp"
#include "ndt/impl/integrated_light_ndt_factor_impl.hpp"

template class gtsam_points::IntegratedLightNDTFactor_<gtsam_points::PointCloud>;

#include <gtsam_points/types/dummy_frame.hpp>
template class gtsam_points::IntegratedLightNDTFactor_<gtsam_points::DummyFrame>;
