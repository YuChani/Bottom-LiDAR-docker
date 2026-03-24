// SPDX-License-Identifier: MIT
// Copyright (c) 2026
//
// Phase 3: IntegratedMixtureLightNDTFactor_ 명시적 템플릿 인스턴스화.
// PointCloud 및 DummyFrame 프레임 타입에 대해 컴파일 유닛 생성.

#include <gtsam_points/types/point_cloud.hpp>
#include "gmm/integrated_mixture_light_ndt_factor.hpp"
#include "gmm/impl/integrated_mixture_light_ndt_factor_impl.hpp"

template class gtsam_points::IntegratedMixtureLightNDTFactor_<gtsam_points::PointCloud>;

#include <gtsam_points/types/dummy_frame.hpp>
template class gtsam_points::IntegratedMixtureLightNDTFactor_<gtsam_points::DummyFrame>;
