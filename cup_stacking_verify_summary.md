# cup_stacking_verify 패키지 요약

> 작성일: 2026-05-18  
> 커밋: `06dfd4e` (최신)

---

## 1. 패키지 개요

`cup_stacking_verify`는 디지털 트윈 환경(`depth_digital_twin`)으로부터 3D 컵 위치를 수신하여 가상 스택 레이아웃과의 겹침(overlap)을 계산하고, 점유 상태를 퍼블리시하며 RViz에서 시각화하는 ROS 2 패키지입니다.

---

## 2. 노드 구성

| 노드 | 실행 파일 | 역할 |
|---|---|---|
| `CupOccupancyNode` | `verifier` | 핵심 점유 검증 노드 |
| `BoxesToDetectionsNode` | `boxes_to_detections` | depth_digital_twin 브리지 |
| `TopicLoggerNode` | `topic_logger` | 토픽 텍스트 모니터 |
| `PoseTunerNode` | `pose_tuner` | p_start/v_dir 실시간 편집 GUI |

---

## 3. 시스템 구조 (데이터 흐름)

```
depth_digital_twin
  └─ /digital_twin/boxes  (MarkerArray, world 프레임, TRANSIENT_LOCAL)
        │
        ▼
  BoxesToDetectionsNode  ──→  /detected_cups  (Detection3DArray)
                                     │
                                     ▼
                             CupOccupancyNode
                              ├─ /cup_occupancy_status  (Int8)
                              ├─ /cup_overlap_ratio     (Float32)
                              └─ /virtual_cup_markers   (MarkerArray)
                                     │
                                     ├─→ RViz2 (cup_verify.rviz)
                                     └─→ TopicLoggerNode (터미널 출력)

PoseTunerNode  ──set_parameters──→  CupOccupancyNode
               (tkinter 슬라이더 UI)
```

---

## 4. verifier_node.py 코드 수정 내역

이전 버전(`bfba1c8`)과 현재 버전(`06dfd4e`) 사이의 주요 변경 사항, 및 추가 튜닝 내역입니다.

### 4-0. 컵 크기 및 레이어 간격 조정 (추가 튜닝)

| 항목 | 변경 전 | 변경 후 |
|---|---|---|
| `cup_ref_w` | 0.078 m | 0.070 m (−10%) |
| `cup_ref_d` | 0.078 m | 0.070 m (−10%) |
| `cup_ref_h` | 0.095 m | 0.086 m (−10%) |
| 레이어 간격 | `cup_ref_h + 0.02` | `cup_ref_h` (간격 제거) |

레이어 높이 계산이 `cup_ref_h`만 사용하므로 레이어 간 추가 공백 없이 바로 인접하여 쌓임.

### 4-1. 새 파라미터 추가

| 파라미터 | 타입 | 기본값 | 설명 |
|---|---|---|---|
| `target_frame` | str | `'world'` | RViz 마커 및 검출 프레임 (기존 하드코딩 `"base_link"` → 설정 가능) |
| `virtual_counts` | List[int] | `[3, 2, 1]` | 레이어별 가상 컵 수 (피라미드 레이아웃, bottom→top) |
| `publish_rate_hz` | float | `10.0` | 마커 퍼블리시 주기 (Hz) |
| `detection_timeout_s` | float | `1.5` | 검출 오버레이 표시 유효 시간 (초) |
| `arrow_length` | float | `0.25` | v_dir 화살표 마커 길이 (m) |

### 4-2. 렌더링 아키텍처 개선 — 타이머 분리

**변경 전**: `detection_callback` 내에서 마커를 즉시 빌드·퍼블리시  
**변경 후**: 콜백은 최신 메시지(`_last_msg`)만 캐싱, 독립 타이머 `_render()`가 주기적으로 마커를 퍼블리시

```python
# 변경 전 (detection_callback 내부에서 직접 퍼블리시)
self.pub_marker.publish(marker_array)

# 변경 후 (타이머 콜백이 항상 렌더)
self._last_msg = msg
self._last_stamp_s = self.get_clock().now().nanoseconds * 1e-9
# ...
self.create_timer(1.0 / rate, self._render)
```

**효과**: 검출이 없어도 가상 경계·포즈 마커가 항상 퍼블리시됨.

### 4-3. `_compute_layers()` 메서드 분리

레이어 그룹화·겹침 계산 로직을 `detection_callback`에서 별도 메서드로 추출.  
`detection_callback`과 `_render()` 양쪽에서 재사용 가능.

### 4-4. 상시 표시 마커 추가

| 메서드 | 마커 타입 | NS | 표시 조건 |
|---|---|---|---|
| `_boundary_outline_marker()` | `LINE_LIST` (와이어프레임) | `virtual_boundary` | 항상 |
| `_pose_markers()` — sphere | `SPHERE` | `pose_origin` | 항상 |
| `_pose_markers()` — arrow | `ARROW` | `pose_dir` | 항상 |
| `_pose_markers()` — label | `TEXT_VIEW_FACING` | `pose_text` | 항상 |

가상 컵 CUBE 마커(점유/미점유)와 검출 컵 CUBE는 `detection_timeout_s` 내 최신 검출이 있을 때만 표시.

### 4-5. `_hdr()` 헬퍼 추가

모든 마커 빌더에서 반복되던 `header.frame_id` / `header.stamp` 설정을 단일 메서드로 통합.

```python
def _hdr(self, marker):
    marker.header.frame_id = self.frame_id
    marker.header.stamp = self.get_clock().now().to_msg()
    return marker
```

### 4-6. `get_virtual_box()` 영벡터 가드 추가

```python
# 변경 전: mag=0이면 ZeroDivisionError 발생
unit_dir = [v_dir[0]/mag, v_dir[1]/mag, v_dir[2]/mag]

# 변경 후
if mag < 1e-9:
    unit_dir = [1.0, 0.0, 0.0]
else:
    unit_dir = [v_dir[0]/mag, v_dir[1]/mag, v_dir[2]/mag]
```

### 4-7. 렌더 사이클 DELETEALL

매 `_render()` 호출 시 `Marker.DELETEALL`로 이전 마커를 먼저 제거하여 RViz 잔상 방지.

### 4-8. 마커 프레임 변경

| 항목 | 변경 전 | 변경 후 |
|---|---|---|
| 모든 마커 `frame_id` | 하드코딩 `"base_link"` | 파라미터 `target_frame` (기본 `"world"`) |

depth_digital_twin이 `world` 프레임으로 박스를 퍼블리시하므로 TF 없이도 마커가 정렬됨.

### 4-9. `main()` 정리

```python
# 변경 전
rclpy.spin(node)
rclpy.shutdown()

# 변경 후
try:
    rclpy.spin(node)
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
```

---

## 5. 새로 추가된 노드

### 5-1. BoxesToDetectionsNode (`boxes_to_detections_node.py`)

`depth_digital_twin`의 `/digital_twin/boxes` (MarkerArray, TRANSIENT_LOCAL QoS)를 수신하여 `verifier`가 소비하는 `/detected_cups` (Detection3DArray)로 변환하는 브리지.

- `ns="boxes"` CUBE 마커 → bbox center + size
- `ns="box_top"` SPHERE 마커 → bbox.center.position.z (컵 top 기준)
- `box_top`이 없으면 `center.z + size.z/2`로 top 추정
- DELETEALL / DELETE 마커 처리로 트랙 소멸 반영

### 5-2. TopicLoggerNode (`topic_logger_node.py`)

파이프라인 전체 토픽을 주기적으로 터미널에 출력하는 디버그 노드.  
구독: `/detected_cups`, `/cup_overlap_ratio`, `/cup_occupancy_status`, `/virtual_cup_markers`  
`period_s` 파라미터(기본 1.0초) 주기로 통합 스냅샷 출력.

### 5-3. PoseTunerNode (`pose_tuner_node.py`)

tkinter 슬라이더 UI로 `cup_occupancy_verifier`의 `p_start` / `v_dir` 파라미터를 실시간 수정.  
`set_parameters` 서비스를 비동기 호출하며, verifier는 매 렌더 틱에 파라미터를 재읽으므로 RViz 경계가 즉시 갱신됨.

---

## 6. 런치 파일 (`cup_verify.launch.py`)

```bash
ros2 launch cup_stacking_verify cup_verify.launch.py
```

| 인자 | 기본값 | 설명 |
|---|---|---|
| `rviz` | `true` | RViz2 실행 여부 |
| `rviz_config` | `rviz/cup_verify.rviz` | RViz 설정 파일 |
| `boxes_topic` | `/digital_twin/boxes` | 입력 MarkerArray 토픽 |
| `detections_topic` | `/detected_cups` | 브리지 출력 / verifier 입력 |
| `target_frame` | `world` | 마커 프레임 |
| `threshold` | `0.6` | 점유 판단 overlap 임계값 |
| `use_test_pub` | `false` | `true` 시 test_publisher로 대체 (standalone 테스트) |
| `tuner` | `true` | pose_tuner UI 실행 여부 |

---

## 7. RViz 마커 NS 정리

| NS | 마커 타입 | 색상 | 표시 조건 |
|---|---|---|---|
| `virtual_boundary` | LINE_LIST | 노란색 (1.0, 0.85, 0.1) | 항상 |
| `pose_origin` | SPHERE | 주황색 (1.0, 0.4, 0.0) | 항상 |
| `pose_dir` | ARROW | 주황색 | 항상 |
| `pose_text` | TEXT | 흰색 | 항상 |
| `virtual_cups` | CUBE | 초록(점유) / 빨강(미점유) | 검출 수신 시 |
| `detected_cups` | CUBE | 파란색 | 검출 수신 시 |
| `overlap_text` | TEXT | 흰색 | 검출 수신 시 |

---

## 8. 실행 방법

```bash
# 빌드
colcon build --packages-select cup_stacking_verify
source install/setup.bash

# 전체 런치 (depth_digital_twin 실행 후)
ros2 launch cup_stacking_verify cup_verify.launch.py

# standalone 테스트 (depth_digital_twin 없이)
ros2 launch cup_stacking_verify cup_verify.launch.py use_test_pub:=true tuner:=false
```
