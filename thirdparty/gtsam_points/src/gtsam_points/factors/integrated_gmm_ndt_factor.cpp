// SPDX-License-Identifier: MIT
// Copyright (c) 2026

#include <gtsam_points/types/point_cloud.hpp>
#include <gtsam_points/factors/integrated_gmm_ndt_factor.hpp>
#include <gtsam_points/factors/impl/integrated_gmm_ndt_factor_impl.hpp>

template class gtsam_points::IntegratedGMMNDTFactor_<gtsam_points::PointCloud>;

#include <gtsam_points/types/dummy_frame.hpp>
template class gtsam_points::IntegratedGMMNDTFactor_<gtsam_points::DummyFrame>;
