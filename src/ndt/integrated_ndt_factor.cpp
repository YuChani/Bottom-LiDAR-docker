// SPDX-License-Identifier: MIT
// Copyright (c) 2021  Kenji Koide (k.koide@aist.go.jp)

#include <gtsam_points/types/point_cloud.hpp>
#include "ndt/integrated_ndt_factor.hpp"
#include "ndt/impl/integrated_ndt_factor_impl.hpp"

template class gtsam_points::IntegratedNDTFactor_<gtsam_points::PointCloud>;

#include <gtsam_points/types/dummy_frame.hpp>
template class gtsam_points::IntegratedNDTFactor_<gtsam_points::DummyFrame>;
