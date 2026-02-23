#pragma once

#include "loam_feature.hpp"
#include <pcl/filters/voxel_grid.h>
#include <algorithm>
#include <cmath>

// LIO-SAM 파라미터 (sensor.yaml에서 가져온 값들)
struct LIOSAMParams
{
    int N_SCAN = 64;                    // LiDAR 채널 수 (Ouster OS1-64)
    int Horizon_SCAN = 1024;            // 수평 해상도
    float edgeThreshold = 0.1;          // Edge 특징점 threshold (Ring-based LOAM과 동일)
    float surfThreshold = 0.1;          // Surface 특징점 threshold
    float odometrySurfLeafSize = 0.4;   // Downsampling leaf size
};

// Cloud Info 구조체 (ROS 메시지 대체)
struct CloudInfo
{
    std::vector<int> startRingIndex;    // 각 Ring의 시작 인덱스
    std::vector<int> endRingIndex;      // 각 Ring의 끝 인덱스
    std::vector<int> pointColInd;       // 각 점의 수평 인덱스
    std::vector<float> pointRange;      // 각 점의 거리
};

struct smoothness_t {
    float value;
    size_t ind;
};

struct by_value {
    bool operator()(smoothness_t const &left, smoothness_t const &right) {
        return left.value < right.value;
    }
};

class FeatureExtraction
{
public:
    // LIO-SAM 파라미터
    LIOSAMParams params;

    // Point Clouds
    pcl::PointCloud<PointXYZIR>::Ptr extractedCloud;
    pcl::PointCloud<PointXYZIR>::Ptr cornerCloud;
    pcl::PointCloud<PointXYZIR>::Ptr surfaceCloud;

    // Downsampling filter
    pcl::VoxelGrid<PointXYZIR> downSizeFilter;

    // Cloud Info (ROS 메시지 대체)
    CloudInfo cloudInfo;

    // Smoothness 관련
    std::vector<smoothness_t> cloudSmoothness;
    float *cloudCurvature;
    int *cloudNeighborPicked;
    int *cloudLabel;

    FeatureExtraction();
    ~FeatureExtraction();

    // 메인 처리 함수 (ROS 콜백 대체)
    LOAMFeatures process(const std::vector<PointWithRing>& points_with_ring);

    // LIO-SAM 파이프라인
    void calculateSmoothness();
    void markOccludedPoints();
    void extractFeatures();

private:
    void initializationValue();
    void convertToPCL(const std::vector<PointWithRing>& points_with_ring);
    void buildCloudInfo();
    LOAMFeatures convertToLOAMFeatures();
};