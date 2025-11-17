# Quick Start Guide

## 빠른 시작 가이드

### 1. 의존성 설치

```bash
# ROS Melodic 환경에서
sudo apt-get update
sudo apt-get install ros-melodic-ar-track-alvar ros-melodic-ar-track-alvar-msgs

# laser_line_extraction 패키지 설치
cd ~/catkin_ws/src
git clone https://github.com/kam3k/laser_line_extraction.git
```

### 2. 패키지 빌드

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 3. 실행

#### 옵션 A: 보정만 실행 (AR 트래킹이 이미 실행 중인 경우)

```bash
roslaunch ar_marker_laser_correction ar_marker_correction.launch
```

#### 옵션 B: 전체 시스템 실행

```bash
# 카메라와 레이저 스캐너가 실행 중이어야 합니다
roslaunch ar_marker_laser_correction complete_system.launch \
    cam_image_topic:=/your_camera/image_raw \
    cam_info_topic:=/your_camera/camera_info \
    laser_topic:=/your_laser/scan \
    marker_size:=10.0
```

### 4. RViz로 확인

```bash
# 자동으로 RViz가 실행되지 않은 경우
rviz -d $(rospack find ar_marker_laser_correction)/config/correction_viz.rviz
```

### 5. 테스트

```bash
# 보정 결과 확인용 스크립트
chmod +x ~/catkin_ws/src/ar_marker_laser_correction/scripts/test_correction.py
rosrun ar_marker_laser_correction test_correction.py
```

## 토픽 확인

```bash
# AR 마커 감지 확인
rostopic echo /ar_pose_marker

# 레이저 직선 확인
rostopic echo /line_segments

# 보정된 마커 확인
rostopic echo /corrected_ar_markers

# 보정된 단일 포즈 확인
rostopic echo /corrected_marker_pose
```

## 파라미터 튜닝 팁

### 환경이 넓은 실내 공간
```bash
roslaunch ar_marker_laser_correction ar_marker_correction.launch \
    search_radius:=1.0 \
    min_line_length:=0.3
```

### 좁고 복잡한 공간
```bash
roslaunch ar_marker_laser_correction ar_marker_correction.launch \
    search_radius:=0.3 \
    angle_tolerance:=10.0 \
    min_line_length:=0.15
```

### 노이즈가 많은 환경
```bash
roslaunch ar_marker_laser_correction ar_marker_correction.launch \
    angle_tolerance:=10.0 \
    min_line_length:=0.4
```

## 문제 해결

### "No line segments received yet" 경고가 계속 나타남
- `laser_line_extraction` 노드가 실행 중인지 확인
- 레이저 스캔 토픽이 올바른지 확인: `rostopic list | grep scan`

### 보정이 적용되지 않음
- 마커가 벽이나 평면 근처에 있는지 확인
- `search_radius` 값을 증가시켜 보세요
- RViz에서 초록색 선(감지된 직선)이 표시되는지 확인

### TF 오류
- `rosrun tf view_frames` 실행
- `base_link`, `laser`, `camera` 프레임이 모두 연결되어 있는지 확인
