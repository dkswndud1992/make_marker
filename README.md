# AR Marker Laser Correction

ROS 1 Melodic 패키지로, AR 마커의 방향을 레이저 스캔 데이터를 활용하여 보정합니다.

## 개요

이 패키지는 AR 마커 인식과 2D 레이저 스캔을 결합하여, 벽이나 표면과 같은 주변 환경의 기하학적 특징을 기반으로 AR 마커의 방향(yaw)을 자동으로 보정합니다. `laser_line_extraction` 패키지를 활용하여 레이저 스캔에서 직선을 추출하고, 마커 근처의 직선과 마커의 방향을 정렬합니다.

## 주요 기능

- AR 마커와 레이저 스캔 데이터를 동기화하여 처리
- 레이저 스캔에서 직선 세그먼트 추출 (`laser_line_extraction` 사용)
- 마커 근처의 직선을 기반으로 방향 보정
- 평행 또는 수직 정렬 자동 감지
- RViz 시각화 지원 (원본 방향, 보정된 방향, 인근 직선)
- 보정된 마커 포즈를 새로운 토픽으로 발행

## 의존성

### ROS 패키지
- `roscpp`
- `rospy`
- `sensor_msgs`
- `geometry_msgs`
- `tf` / `tf2`
- `ar_track_alvar` (AR 마커 인식)
- `laser_line_extraction` (레이저 직선 추출)
- `visualization_msgs`

### 설치 방법

```bash
# 의존성 패키지 설치
sudo apt-get install ros-melodic-ar-track-alvar ros-melodic-ar-track-alvar-msgs

# laser_line_extraction 설치
cd ~/catkin_ws/src
git clone https://github.com/kam3k/laser_line_extraction.git
cd ~/catkin_ws
catkin_make

# 이 패키지 빌드
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 사용법

### 기본 실행

보정 노드만 실행 (AR 트래킹과 레이저 스캔이 이미 실행 중인 경우):

```bash
roslaunch ar_marker_laser_correction ar_marker_correction.launch
```

### 완전한 시스템 실행

AR 트래킹과 보정을 함께 실행:

```bash
roslaunch ar_marker_laser_correction complete_system.launch
```

### 주요 파라미터

Launch 파일에서 다음 파라미터를 조정할 수 있습니다:

```xml
<arg name="base_frame" default="base_link" />          <!-- 기준 프레임 -->
<arg name="search_radius" default="0.2" />             <!-- 직선 탐색 반경 (미터) -->
<arg name="angle_tolerance" default="45.0" />          <!-- 각도 허용 오차 (도) -->
<arg name="min_line_length" default="0.05" />           <!-- 최소 직선 길이 (미터) -->
<arg name="publish_visualization" default="true" />    <!-- 시각화 발행 여부 -->
```

## 노드 정보

### ar_marker_laser_correction_node

#### 구독 토픽
- `/ar_pose_marker` (`ar_track_alvar_msgs/AlvarMarkers`): AR 마커 포즈
- `/line_segments` (`laser_line_extraction/LineSegmentList`): 추출된 레이저 직선

#### 발행 토픽
- `/corrected_marker_pose` (`geometry_msgs/PoseStamped`): 보정된 마커 포즈
- `/corrected_ar_markers` (`ar_track_alvar_msgs/AlvarMarkers`): 보정된 마커 배열
- `/correction_visualization` (`visualization_msgs/MarkerArray`): 시각화 마커

#### 파라미터
- `~base_frame` (string, default: "base_link"): 기준 좌표계
- `~search_radius` (double, default: 0.5): 마커 주변 직선 탐색 반경 (m)
- `~angle_tolerance` (double, default: 15.0): 각도 정렬 허용 오차 (도)
- `~min_line_length` (double, default: 0.2): 고려할 최소 직선 길이 (m)
- `~publish_visualization` (bool, default: true): 시각화 마커 발행 여부

## 알고리즘

1. **AR 마커 감지**: `ar_track_alvar`를 통해 AR 마커의 포즈를 획득
2. **레이저 직선 추출**: `laser_line_extraction`을 통해 레이저 스캔에서 직선 세그먼트 추출
3. **근접 직선 탐색**: 마커 위치에서 `search_radius` 내의 직선 필터링
4. **방향 보정**: 
   - 마커와 평행한 직선 찾기
   - 마커와 수직인 직선 찾기
   - 가장 가까운 정렬 각도 선택
5. **보정된 포즈 발행**: 위치는 유지하고 방향(yaw)만 업데이트

## 시각화

RViz에서 다음 항목을 확인할 수 있습니다:

- **빨간 화살표**: 원본 AR 마커 방향
- **파란 화살표**: 보정된 마커 방향
- **초록 선**: 마커 근처에서 감지된 레이저 직선

RViz 설정 파일은 `config/correction_viz.rviz`에 포함되어 있습니다.

## 예제 사용 시나리오

### 시나리오 1: 벽에 부착된 마커

벽에 부착된 AR 마커의 경우, 벽의 직선을 감지하여 마커가 벽과 평행하도록 방향을 자동 보정합니다.

### 시나리오 2: 모서리 근처의 마커

모서리 근처에 있는 마커는 두 벽의 직선을 모두 사용하여 더 정확한 방향 추정이 가능합니다.

## 설정 팁

### 레이저 직선 추출 튜닝

`config/line_extraction_params.yaml` 파일을 수정하여 직선 추출 품질을 조정할 수 있습니다:

```yaml
min_line_points: 9          # 직선을 구성하는 최소 포인트 수
min_line_length: 0.2        # 최소 직선 길이 (m)
outlier_dist: 0.05          # 이상치 거리 임계값
```

### 보정 민감도 조정

- **search_radius 증가**: 더 먼 직선도 고려 (환경이 넓은 경우)
- **angle_tolerance 증가**: 더 큰 각도 차이 허용 (노이즈가 많은 경우)
- **min_line_length 증가**: 더 긴 직선만 사용 (짧은 세그먼트 무시)

## 문제 해결

### 보정이 작동하지 않는 경우

1. 레이저 스캔이 마커 근처의 벽/표면을 충분히 감지하는지 확인
2. TF 트리가 올바르게 설정되었는지 확인 (`rosrun tf view_frames`)
3. RViz에서 직선이 제대로 추출되는지 확인
4. `search_radius` 파라미터 증가

### 노이즈가 많은 경우

1. `angle_tolerance` 감소
2. `min_line_length` 증가
3. 레이저 스캔 품질 개선 (필터 적용)

## 라이선스

MIT License

## 기여

버그 리포트나 기능 제안은 GitHub Issues를 통해 제출해 주세요.

## 참고

- [ar_track_alvar](http://wiki.ros.org/ar_track_alvar)
- [laser_line_extraction](https://github.com/kam3k/laser_line_extraction)
