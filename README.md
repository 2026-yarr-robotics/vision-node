# vision-node — 컵 점유 검증 (depth_digital_twin 연동)

`cup_stacking_verify` 패키지. **depth_digital_twin**(`ros2-depth-point-cloude`)
파이프라인이 검출한 실제 컵의 3D 포즈를 받아, 목표 적재 위치의 **가상 컵**과
겹침 비율(overlap)을 계산하여 점유 여부를 판정하고 RViz로 시각화한다.

두 프로젝트는 **합치지 않고** ROS2 토픽으로 연결한다. RViz 창도 **각각 따로**
뜬다 (depth_digital_twin 창 1개 + vision-node 창 1개).

---

## 시스템 구성 / 데이터 흐름

```
[ros2-depth-point-cloude / depth_digital_twin]        [vision-node / cup_stacking_verify]

 camera(RealSense) ─┐
 robot(dsr_bringup) ─┤
                     ▼
 detection_node ─► /digital_twin/detections
                     ▼
 point_cloud_node ─► /digital_twin/boxes ──────────►  boxes_to_detections_node
   (MarkerArray, world frame, latched)                  │  (MarkerArray → Detection3DArray)
                     ▼                                   ▼
 pick_ui_node                                       /detected_cups (vision_msgs/Detection3DArray)
                                                         ▼
 RViz #1 (digital_twin.rviz)                        cup_occupancy_verifier
   points / boxes / debug image                       │  ├─► /cup_occupancy_status (Int8)
                                                       │  ├─► /cup_overlap_ratio   (Float32)
                                                       │  └─► /virtual_cup_markers (MarkerArray)
                                                       ▼
                                                  topic_logger_node  ─► 텍스트 로그
                                                       ▼
                                                  RViz #2 (cup_verify.rviz)
                                                    virtual / detected cups
```

- depth_digital_twin 은 `world`(= 로봇 베이스) 프레임으로 박스를 퍼블리시한다.
- 브리지/검증 노드도 기본 `world` 프레임을 쓰므로 `world↔base_link` TF 없이
  두 RViz 창의 형상이 일치한다. (`target_frame:=base_link` 로 변경 가능)

---

## 노드 구성 (`cup_stacking_verify`)

| 실행파일 | 노드 | 역할 |
|---|---|---|
| `boxes_to_detections` | `boxes_to_detections_node` | **브리지.** `/digital_twin/boxes`(MarkerArray) → `/detected_cups`(Detection3DArray). `boxes` CUBE → 크기/중심, `box_top` 구 → 윗면 중심. DELETE/DELETEALL 처리. |
| `verifier` | `cup_occupancy_verifier` | `/detected_cups` 를 받아 가상 컵과 overlap 계산, 점유 판정 + 마커 퍼블리시. |
| `topic_logger` | `topic_logger_node` | vision-node 가 퍼블리시하는 토픽을 1초마다 텍스트로 출력. |
| `test_publisher` | `test_publisher` | depth_digital_twin 없이 단독 테스트용 합성 3-2-1 피라미드 퍼블리셔. |

---

## 빌드

**depth_digital_twin (한 번):**

```bash
cd ~/Projects/ros2-depth-point-cloude
colcon build
source install/setup.bash
```

**vision-node:**

```bash
cd ~/Projects/vision-node
colcon build --packages-select cup_stacking_verify
source install/setup.bash
```

> 스테일 `install/` 가 남아 `--symlink-install` 빌드가 실패하면:
> `rm -rf build/cup_stacking_verify install/cup_stacking_verify` 후 다시 빌드.

---

## 실행 방법 (전체 통합 — depth_digital_twin 모듈 포함)

depth_digital_twin 을 터미널 1~4 순서대로 먼저 띄우고, 마지막에 vision-node
를 띄운다. 각 터미널은 해당 워크스페이스를 `source` 한 상태여야 한다.

```bash
# 터미널 1: 로봇
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py \
    mode:=real model:=m0609 host:=192.168.1.100

# 터미널 2: 카메라
ros2 launch realsense2_camera rs_align_depth_launch.py \
    depth_module.depth_profile:=1280x720x30 \
    rgb_camera.color_profile:=1280x720x30 \
    initial_reset:=true align_depth.enable:=true

# 터미널 3: detection (depth_digital_twin RViz #1 포함)
ros2 launch depth_digital_twin digital_twin.launch.py

# 터미널 4: pick UI
ros2 run depth_digital_twin pick_ui_node

# 터미널 5: vision-node (브리지 + 검증 + 로거 + RViz #2)
ros2 launch cup_stacking_verify cup_verify.launch.py
```

터미널 5 실행 시 별도의 RViz 창(#2)이 뜨고, depth_digital_twin 이 검출한
컵(`/digital_twin/boxes`)이 자동으로 `/detected_cups` 로 변환되어 검증된다.

### 단독 테스트 (depth_digital_twin 없이)

로봇/카메라 없이 검증 로직만 확인할 때 — 합성 퍼블리셔로 대체:

```bash
ros2 launch cup_stacking_verify cup_verify.launch.py use_test_pub:=true
```

### launch 인자

| 인자 | 기본값 | 설명 |
|---|---|---|
| `rviz` | `true` | vision-node RViz 창 띄우기 |
| `rviz_config` | 패키지 `rviz/cup_verify.rviz` | RViz 설정 파일 |
| `boxes_topic` | `/digital_twin/boxes` | depth_digital_twin 입력 토픽 |
| `detections_topic` | `/detected_cups` | 브리지 출력 토픽 |
| `target_frame` | `world` | 마커/검출 프레임 (`base_link` 로 변경 가능) |
| `threshold` | `0.6` | 점유 판정 overlap 임계값 |
| `use_test_pub` | `false` | `true` → 브리지 대신 `test_publisher` 실행 |

---

## 토픽 / 프레임

**구독**

- `/digital_twin/boxes` (`visualization_msgs/MarkerArray`) — 브리지 입력
  (depth_digital_twin, latched/transient-local QoS, `world` 프레임)
- `/detected_cups` (`vision_msgs/Detection3DArray`) — 검증 노드 입력
  (브리지 출력. 컨벤션: `bbox.center.position.z` = 컵 **윗면**, 컵 범위
  `[z - size.z, z]`)

**퍼블리시 (검증 노드)**

- `/virtual_cup_markers` (`visualization_msgs/MarkerArray`) — 가상/검출 컵,
  overlap 텍스트 마커
- `/cup_occupancy_status` (`std_msgs/Int8`) — 점유 1 / 비점유 0
- `/cup_overlap_ratio` (`std_msgs/Float32`) — 최대 overlap 비율

**RViz 마커 색상**: 검출 컵 파랑, 가상 컵 점유 시 초록·미점유 시 빨강,
overlap 값은 흰색 텍스트. 모든 마커는 `target_frame`(기본 `world`).

---

## 발행 토픽 텍스트로 확인

`topic_logger_node` 가 터미널 5 로그에 1초마다 요약 출력한다:

```
──── vision-node topics ────
/detected_cups: 6 cup(s)
    #  1  top=(+0.500, +0.000, +0.195)
    ...
/cup_overlap_ratio: max=0.842
/cup_occupancy_status: last=1 (OCCUPIED)  msgs=120
/virtual_cup_markers: 5/6 virtual cup(s) occupied
```

개별 토픽을 직접 보고 싶으면:

```bash
ros2 topic echo /cup_overlap_ratio
ros2 topic echo /cup_occupancy_status
ros2 topic echo /detected_cups --no-arr
```

---

## 파라미터 (`cup_occupancy_verifier`)

| 파라미터 | 타입 | 기본값 | 설명 |
|---|---|---|---|
| `p_start` | float[3] | `[0.5, 0.0, 0.1]` | 가상 컵 적재 시작 위치 |
| `v_dir` | float[3] | `[1.0, 0.0, 0.0]` | 가상 컵 정렬 방향 벡터 |
| `threshold` | float | `0.6` | 점유 판정 overlap 임계값 |
| `target_frame` | string | `world` | 모든 마커의 프레임 |
| `target_index` | int | `0` | 타겟 가상 컵 인덱스 |

검증 노드는 z 로 레이어를 묶고 각 레이어 내 x 정렬 후 가상 피라미드
(3-2-1 …)와 비교한다.

---

## 처리 요약

1. `boxes_to_detections_node` — `/digital_twin/boxes` 스냅샷에서 id별 컵 복원
   → `/detected_cups` 재퍼블리시.
2. `cup_occupancy_verifier` — 레이어 그룹화 → 가상 박스와 overlap(IoU 부피비)
   계산 → 점유 판정 → 마커/상태/비율 퍼블리시.
3. `topic_logger_node` — 위 토픽을 텍스트로 주기 출력.

---

## 의존성

- ROS2 (rclpy), `vision_msgs`, `visualization_msgs`, `std_msgs`, `geometry_msgs`
- RViz2, `ros2launch`
- depth_digital_twin (`ros2-depth-point-cloude`) — 실제 검출 입력 제공
- 로봇/카메라: `dsr_bringup2`(Doosan M0609), `realsense2_camera`
