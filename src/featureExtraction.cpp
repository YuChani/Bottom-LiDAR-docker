#include "featureExtraction.hpp"
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/filters/impl/filter.hpp>
#include <spdlog/spdlog.h>

// yuchan summary : Feature Extraction 부분(LOAM 기반 - LIO-SAM 원본)
// calculateSmoothness() : points의 양옆 5개씩 10개의 점과의 거리 차이 합산으로 curvature 계산
// markOccludedPoints() : 벽 뒤에 가려진 점들 마킹(예를 틀어서 장애물 뒤에 찍힌 point들을 마킹해서 제외)
// extractFeatures() : 각 scan을 6등분하고 이 섹터별로 smoothness 기준으로 오름차순 정렬 후 edge, surface 특징점 추출

FeatureExtraction::FeatureExtraction()
{
    initializationValue();
}

FeatureExtraction::~FeatureExtraction()
{
    delete[] cloudCurvature;
    delete[] cloudNeighborPicked;
    delete[] cloudLabel;
}

void FeatureExtraction::initializationValue()
{
    cloudSmoothness.resize(params.N_SCAN * params.Horizon_SCAN);

    downSizeFilter.setLeafSize(params.odometrySurfLeafSize, params.odometrySurfLeafSize, params.odometrySurfLeafSize);

    extractedCloud.reset(new pcl::PointCloud<PointXYZIR>());
    cornerCloud.reset(new pcl::PointCloud<PointXYZIR>());
    surfaceCloud.reset(new pcl::PointCloud<PointXYZIR>());

    cloudCurvature = new float[params.N_SCAN * params.Horizon_SCAN];
    cloudNeighborPicked = new int[params.N_SCAN * params.Horizon_SCAN];
    cloudLabel = new int[params.N_SCAN * params.Horizon_SCAN];
}

// yuchan : 메인 처리 함수 (ROS 콜백 대체)
LOAMFeatures FeatureExtraction::process(const std::vector<PointWithRing>& points_with_ring)
{
    // 1. PointWithRing → PCL PointCloud 변환
    convertToPCL(points_with_ring);

    // 2. Cloud Info 생성
    buildCloudInfo();

    // 3. LIO-SAM 파이프라인
    calculateSmoothness();
    markOccludedPoints();
    extractFeatures();

    // 4. LOAMFeatures로 변환하여 반환
    return convertToLOAMFeatures();
}

// yuchan : PointWithRing → PCL PointCloud 변환
void FeatureExtraction::convertToPCL(const std::vector<PointWithRing>& points_with_ring)
{
    extractedCloud->clear();
    extractedCloud->reserve(points_with_ring.size());

    for (const auto& p : points_with_ring)
    {
        PointXYZIR pt;
        pt.x = p.point.x();
        pt.y = p.point.y();
        pt.z = p.point.z();
        pt.intensity = 0.0f;  // Intensity는 사용 안 함
        pt.ring = p.ring;
        extractedCloud->push_back(pt);
    }

    spdlog::info("Converted {} points to PCL format", extractedCloud->size());
}

// yuchan : CloudInfo 생성 (ROS 메시지 대체)
void FeatureExtraction::buildCloudInfo()
{
    int cloudSize = extractedCloud->size();
    // Step 1: 각 ring별로 포인트 인덱스 수집
    std::vector<std::vector<int>> ringIndices(params.N_SCAN);
    for (int i = 0; i < cloudSize; ++i)
    {
        int ring = extractedCloud->points[i].ring;
        if (ring < 0 || ring >= params.N_SCAN)
            continue;
        ringIndices[ring].push_back(i);
    }

    // Step 2: 각 ring 내에서 angle(atan2) 기반 정렬 후 extractedCloud를 재배치
    // Ring-based LOAM과 동일하게 회전 순서대로 정렬하여
    // curvature 계산 시 인접 포인트가 실제 공간상의 이웃이 되도록 보장
    pcl::PointCloud<PointXYZIR>::Ptr sortedCloud(new pcl::PointCloud<PointXYZIR>());
    sortedCloud->reserve(cloudSize);

    for (int ring = 0; ring < params.N_SCAN; ring++)
    {
        auto& indices = ringIndices[ring];
        if (indices.empty()) continue;

        // angle 기반 정렬 (atan2(y, x))
        std::sort(indices.begin(), indices.end(), [this](int a, int b) {
            const auto& pa = extractedCloud->points[a];
            const auto& pb = extractedCloud->points[b];
            return std::atan2(pa.y, pa.x) < std::atan2(pb.y, pb.x);
        });

        // 정렬된 순서로 포인트 추가
        for (int idx : indices)
        {
            sortedCloud->push_back(extractedCloud->points[idx]);
        }
    }

    // 정렬된 클라우드로 교체
    extractedCloud = sortedCloud;
    cloudSize = extractedCloud->size();

    // Step 3: 정렬된 클라우드에서 CloudInfo 빌드
    cloudInfo.startRingIndex.assign(params.N_SCAN, -1);
    cloudInfo.endRingIndex.assign(params.N_SCAN, -1);
    cloudInfo.pointColInd.resize(cloudSize);
    cloudInfo.pointRange.resize(cloudSize);
    // Ring별 포인트 카운터 (수평 인덱스 계산용)
    std::vector<int> ringPointCount(params.N_SCAN, 0);
    // Ring별 시작/끝 인덱스 계산
    for (int i = 0; i < cloudSize; ++i)
    {
        int ring = extractedCloud->points[i].ring;
        if (ring < 0 || ring >= params.N_SCAN)
            continue;
        if (cloudInfo.startRingIndex[ring] == -1)
            cloudInfo.startRingIndex[ring] = i;
        cloudInfo.endRingIndex[ring] = i;
        // 수평 인덱스 계산 (Ring 내 순차 인덱스 사용)
        cloudInfo.pointColInd[i] = ringPointCount[ring];
        ringPointCount[ring]++;
        // Range 계산
        const auto& pt = extractedCloud->points[i];
        cloudInfo.pointRange[i] = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
    }

    spdlog::info("Built CloudInfo for {} rings (angle-sorted)", params.N_SCAN);
}

// yuchan : LOAMFeatures로 변환
LOAMFeatures FeatureExtraction::convertToLOAMFeatures()
{
    LOAMFeatures features;
    features.edge_points = std::make_shared<gtsam_points::PointCloudCPU>();
    features.planar_points = std::make_shared<gtsam_points::PointCloudCPU>();

    std::vector<Eigen::Vector4d> edge_pts, planar_pts;

    // Corner points → Edge points
    for (const auto& pt : cornerCloud->points)
    {
        edge_pts.emplace_back(pt.x, pt.y, pt.z, 1.0);
    }

    // Surface points → Planar points
    for (const auto& pt : surfaceCloud->points)
    {
        planar_pts.emplace_back(pt.x, pt.y, pt.z, 1.0);
    }

    features.edge_points->add_points(edge_pts);
    features.planar_points->add_points(planar_pts);

    spdlog::info("LIO-SAM features: {} edge, {} planar", edge_pts.size(), planar_pts.size());

    return features;
}

// yuchan : curvature 계산
// calculate smoothness(곡률계산 LOAM에서는 c값)
// Ring 경계를 넘지 않도록 startRingIndex+5 ~ endRingIndex-5 범위에서만 계산
void FeatureExtraction::calculateSmoothness()
{
    int cloudSize = extractedCloud->points.size();
    // Ring 경계 집합 생성 — ring 시작/끝 근처 5개 포인트는 curvature 계산에서 제외
    // (다른 ring의 포인트와 섞여서 잘못된 curvature가 계산되는 것을 방지)
    std::vector<bool> ringBoundary(cloudSize, false);
    for (int ring = 0; ring < params.N_SCAN; ring++)
    {
        int start = cloudInfo.startRingIndex[ring];
        int end = cloudInfo.endRingIndex[ring];
        if (start < 0 || end < 0) continue;

        // ring 시작 근처 5개
        for (int k = start; k <= std::min(start + 4, end); k++)
            ringBoundary[k] = true;
        // ring 끝 근처 5개
        for (int k = std::max(end - 4, start); k <= end; k++)
            ringBoundary[k] = true;
    }
    // curvature 계산에서 양옆 5개씩 총 10개의 점의 거리 차이 합산.
    for (int i = 5; i < cloudSize - 5; i++)
    {
        // Ring 경계 근처 포인트는 건너뜀 (다른 ring 포인트와의 혼합 방지)
        if (ringBoundary[i])
        {
            cloudCurvature[i] = 0;
            cloudNeighborPicked[i] = 1;  // 제외 마킹
            cloudLabel[i] = 0;
            cloudSmoothness[i].value = 0;
            cloudSmoothness[i].ind = i;
            continue;
        }
        float diffRange = cloudInfo.pointRange[i-5] + cloudInfo.pointRange[i-4]
                        + cloudInfo.pointRange[i-3] + cloudInfo.pointRange[i-2]
                        + cloudInfo.pointRange[i-1] - cloudInfo.pointRange[i] * 10
                        + cloudInfo.pointRange[i+1] + cloudInfo.pointRange[i+2]
                        + cloudInfo.pointRange[i+3] + cloudInfo.pointRange[i+4]
                        + cloudInfo.pointRange[i+5];
        // 제곱해서 (-) 부호 제거
        cloudCurvature[i] = diffRange*diffRange;    //diffX * diffX + diffY * diffY + diffZ * diffZ;
        cloudNeighborPicked[i] = 0;
        cloudLabel[i] = 0;
        // cloudSmoothness for sorting
        cloudSmoothness[i].value = cloudCurvature[i];
        cloudSmoothness[i].ind = i;
    }
}

// yuchan : occluded points 마킹
// LOAM 논문에서 벽뒤에 가려진 포인트가 찍힐수 있으니 그 부분을 마킹해서 제외시킴
// Ring 경계를 넘지 않도록 같은 ring 내에서만 비교
void FeatureExtraction::markOccludedPoints()
{
    int cloudSize = extractedCloud->points.size();
    // mark occluded points and parallel beam points
    for (int i = 5; i < cloudSize - 6; ++i)
    {
        // Ring 경계 체크: i와 i+1이 같은 ring인지 확인
        int ring_i = extractedCloud->points[i].ring;
        int ring_i1 = extractedCloud->points[i+1].ring;
        if (ring_i != ring_i1)
            continue;  // 다른 ring이면 건너뜀
        // occluded points
        float depth1 = cloudInfo.pointRange[i];
        float depth2 = cloudInfo.pointRange[i+1];
        int columnDiff = std::abs(int(cloudInfo.pointColInd[i+1] - cloudInfo.pointColInd[i]));
        // 10 픽셀인데 depth가 0.3 이상이면 occlusion point로 생각
        if (columnDiff < 10){
            // 10 pixel diff in range image
            // cloudNeighborPicked이 1이되면 해당 점은 제외시키는 마킹
            // depth1(왼쪽점)이 depth2(오른쪽점)보다 작으면 왼쪽점이 앞에 있는거니까 오른쪽 점들은 벽 뒤에 가려진 점들 
            if (depth1 - depth2 > 0.3)
            {
                cloudNeighborPicked[i - 5] = 1;
                cloudNeighborPicked[i - 4] = 1;
                cloudNeighborPicked[i - 3] = 1;
                cloudNeighborPicked[i - 2] = 1;
                cloudNeighborPicked[i - 1] = 1;
                cloudNeighborPicked[i] = 1;
            }
            // depth2(오른쪽점)이 depth1(왼쪽점)보다 작으면 오른쪽점이 앞에 있는거니까 왼쪽 점들은 벽 뒤에 가려진 점들
            else if (depth2 - depth1 > 0.3)
            {
                cloudNeighborPicked[i + 1] = 1;
                cloudNeighborPicked[i + 2] = 1;
                cloudNeighborPicked[i + 3] = 1;
                cloudNeighborPicked[i + 4] = 1;
                cloudNeighborPicked[i + 5] = 1;
                cloudNeighborPicked[i + 6] = 1;
            }
        }
        // parallel beam
        float diff1 = std::abs(float(cloudInfo.pointRange[i-1] - cloudInfo.pointRange[i]));
        float diff2 = std::abs(float(cloudInfo.pointRange[i+1] - cloudInfo.pointRange[i]));
        if (diff1 > 0.02 * cloudInfo.pointRange[i] && diff2 > 0.02 * cloudInfo.pointRange[i])
            cloudNeighborPicked[i] = 1; // 1이면 제외시키는 마킹
    }
}

// yuchan : 특징점 추출 부분
// curvature가지고 edge랑 surface(planar) 특징점 추출하고 6개의 구역으로 나눔
// cloudLabel = -1 : surface point, cloudLabel = 1 : edge point, cloudLabel = 0 : 일반 point
// 6등분 이유 -> 균등하게 분포해서 특징점 추출하려고 / 각 섹터마다 최대 20개의 edge point 추출
// 20개 제한 -> 너무 많은 edge point가 한쪽에 몰리는걸 방지 / 균등하게 분포시키기 위해
// downsampling도 수행(surface points에 대해서)
void FeatureExtraction::extractFeatures()
{
    cornerCloud->clear();   // edge points
    surfaceCloud->clear(); // surface points

    pcl::PointCloud<PointXYZIR>::Ptr surfaceCloudScan(new pcl::PointCloud<PointXYZIR>());
    pcl::PointCloud<PointXYZIR>::Ptr surfaceCloudScanDS(new pcl::PointCloud<PointXYZIR>());
    // 각 scan 별로 특정점 추출
    for (int i = 0; i < params.N_SCAN; i++)
    {
        surfaceCloudScan->clear();  // 각 scan 별로 surface points 저장

        // 각 scan을 6등분하여 특징점 추출 why? -> 균등하게 분포해서 추출하려고
        for (int j = 0; j < 6; j++)
        {
            // edge랑 surface 특정점 point 추출 범위 설정
            int sp = (cloudInfo.startRingIndex[i] * (6 - j) + cloudInfo.endRingIndex[i] * j) / 6;
            int ep = (cloudInfo.startRingIndex[i] * (5 - j) + cloudInfo.endRingIndex[i] * (j + 1)) / 6 - 1;

            if (sp >= ep)
                continue;
            // smoothness 기준으로 오름차순 정렬
            std::sort(cloudSmoothness.begin()+sp, cloudSmoothness.begin()+ep, by_value()); 

            int largestPickedNum = 0;
            // edge points 추출하는데, curvature가 큰 순서대로 2개까지 추출 (Ring-based LOAM과 동일)
            // 뒤에서 부터(곡률 큰 순서)
            for (int k = ep; k >= sp; k--)
            {
                int ind = cloudSmoothness[k].ind;   // 실제 point 인덱스 
                        // edge point 조건 : 아직 선택되지 않은 점 && curvature가 threshold 이상
                // 이미 선택된 이웃점이 아니고, 곡률이 threshold 보다 크면 
                if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > params.edgeThreshold)
                {
                    largestPickedNum++;
                    if (largestPickedNum <= 2) // 최대 2개까지 edge point로 선택 (Ring-based LOAM과 동일)
                    {
                        cloudLabel[ind] = 1;
                        cornerCloud->push_back(extractedCloud->points[ind]);    // edge point 저장
                    } 
                    else 
                    {
                        break;  // 2개 넘으면 종료
                    }
                    // 선택된 point 양옆5개 이웃점들은 제외시키기 위해 마킹
                    cloudNeighborPicked[ind] = 1;
                    // 오른쪽 5개 이웃점 제외
                    for (int l = 1; l <= 5; l++)
                    {
                        int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
                        if (columnDiff > 10) // 점들이 멀리 떨어져있으면 break
                            break;
                        cloudNeighborPicked[ind + l] = 1;
                    }
                    // 왼쪽 5개 이웃점 제외
                    for (int l = -1; l >= -5; l--)
                    {
                        int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
                        if (columnDiff > 10)
                            break;
                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }
            // 곡률이 작은 점들은 surface points 후보로 마킹 (최대 4개, Ring-based LOAM과 동일)
            int surfPickedNum = 0;
            for (int k = sp; k <= ep; k++) // 앞에서 부터 (곡률 작은 순서)
            {
                int ind = cloudSmoothness[k].ind;
                // Surface point 조건: 선택안됨 && 곡률 낮음
                if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < params.surfThreshold)
                {
                    surfPickedNum++;
                    if (surfPickedNum > 4) // 최대 4개까지 surface point로 선택
                        break;
                    cloudLabel[ind] = -1;   // surface point 라벨링
                    cloudNeighborPicked[ind] = 1;
                    // 오른쪽 5개 이웃점 제외시키기 위해 마킹 edge랑 비슷한 방식
                    for (int l = 1; l <= 5; l++) 
                    {
                        int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
                        if (columnDiff > 10)
                            break;

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    // 왼쪽 5개 이웃점 제외시키기 위해 마킹
                    for (int l = -1; l >= -5; l--) 
                    {
                        int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
                        if (columnDiff > 10)
                            break;

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }
            // cloudLabel == -1인 점들만 surface point로 저장 (명시적으로 라벨링된 surface만)
            for (int k = sp; k <= ep; k++)
            {
                if (cloudLabel[cloudSmoothness[k].ind] == -1){
                    surfaceCloudScan->push_back(extractedCloud->points[cloudSmoothness[k].ind]);
                }
            }
        }

        surfaceCloudScanDS->clear();
        downSizeFilter.setInputCloud(surfaceCloudScan);
        downSizeFilter.filter(*surfaceCloudScanDS);

        *surfaceCloud += *surfaceCloudScanDS;
    }
}
