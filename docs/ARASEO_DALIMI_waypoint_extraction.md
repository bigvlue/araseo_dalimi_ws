# ARASEO-DALIMI Waypoint Extraction — 기술 정리

> 작성 기준: 2026-04  
> 대상 트랙: ARASEO-DALIMI 미니시티 실내 트랙 (3840×2160 px 항공 이미지)  
> 추출 결과: 14개 경로, 총 2,435 waypoints

---

## 1. 작업 개요

탑뷰(Bird's-Eye View) 트랙 이미지에서 **Magenta 색상으로 표시된 경로**를 자동 추출하여 Pure Pursuit 알고리즘에 바로 사용 가능한 waypoint 배열 JSON을 생성하는 파이프라인을 구축했다.

### 입력 파라미터

| 파라미터 | 값 |
|---|---|
| 추출 색상 | Magenta 레이어 (HSV 기반) |
| 샘플링 간격 | 20 px |
| Douglas-Peucker ε | 0 (단순화 없음) |
| 대상 해상도 | 3840 × 2160 px |
| 좌표계 | 이미지 픽셀, 원점 좌상단, x→우, y→하 |

---

## 2. 핵심 파이프라인

```
이미지 로드 & 리사이즈 (→ 3840×2160)
    ↓
HSV 마스킹 (Magenta 색상 추출)
    ↓
모폴로지 정리 (MORPH_CLOSE)
    ↓
연결 컴포넌트 분석 → 대상 컴포넌트 선택
    ↓
골격화 (skeletonize)
    ↓
경로 정렬 (방식은 경로 유형에 따라 선택)
    ↓
20px 등간격 선형 보간 샘플링
    ↓
JSON 출력
```

---

## 3. HSV 마스킹

```python
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
m1  = cv2.inRange(hsv, (140, 100, 100), (180, 255, 255))  # Hue 140~180
m2  = cv2.inRange(hsv, (0,  100, 100),  (8,   255, 255))  # Hue 0~8 (wrap-around)
mask = cv2.bitwise_or(m1, m2)
```

- Magenta는 HSV Hue가 140~180 구간과 0~10 구간(wrap-around)에 걸쳐 있다
- Saturation/Value 임계값 100으로 저채도 노이즈 제거
- 이미지 외곽 30px border를 0으로 마스킹하여 경계 노이즈 차단

---

## 4. 컴포넌트 선택 전략

가장 큰 컴포넌트가 항상 목표 경로가 아닌 경우가 있다. 다음 기준으로 대상 컴포넌트를 선택한다.

```python
_, labels = cv2.connectedComponents(mask)
cnt_arr = np.bincount(labels.ravel()); cnt_arr[0] = 0
sorted_ids = np.argsort(cnt_arr)[::-1]

# 공간 필터 조건 예시 (우하단 소형 루프)
for comp_id in sorted_ids:
    ys, xs = np.where(labels == comp_id)
    cx = (xs.min() + xs.max()) // 2
    cy = (ys.min() + ys.max()) // 2
    w  = xs.max() - xs.min()
    h  = ys.max() - ys.min()
    if cx > 1800 and cy > 1100 and w < 1400 and h < 900:
        target_id = comp_id; break
```

- 전체 이미지를 걸친 노이즈 컴포넌트(w>2000 or h>1500) 제외
- 경로의 예상 위치(cx, cy)와 크기(w, h)로 공간 필터링
- 노이즈 제외 후 남은 컴포넌트 중 가장 큰 것 선택

---

## 5. 경로 정렬 방식 (3가지)

골격화된 픽셀을 **순서 있는 경로**로 정렬하는 방식은 경로 유형에 따라 달라진다.

### 5-1. findContours 방식 (기본, ratio < 1.5)

```python
skel_d = cv2.dilate(skel, np.ones((3,3), np.uint8), iterations=1)
contours, _ = cv2.findContours(skel_d, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
raw = sorted(contours, key=len, reverse=True)[0].reshape(-1, 2).astype(np.float64)
```

- 1px skeleton을 3px dilate 후 외곽 컨투어 추출
- OpenCV findContours가 픽셀 순서를 자동으로 정렬해줌
- **판별 기준**: `contour 길이 / skeleton 픽셀 수` 비율(ratio) < 1.5 이면 단일 경로로 판단

### 5-2. KDTree NN 정렬 (ratio ≈ 2.0, hollow ring 구조)

```python
ys, xs = np.where(skel > 0)
pts = np.column_stack([xs, ys])

remaining = pts.copy()
ordered = [remaining[0]]; remaining = remaining[1:]
while len(remaining) > 0:
    _, idx = KDTree(remaining).query(ordered[-1])
    ordered.append(remaining[idx])
    remaining = np.delete(remaining, idx, axis=0)
```

- Hollow(두 줄 선) skeleton에서 contour가 왕복 2배로 감지될 때 사용
- 골격 픽셀을 직접 최근접 이웃(NN) 체인으로 정렬
- 대용량 시 `pts[::STEP]` 다운샘플 후 정렬 (STEP = skel_px // 2000)

### 5-3. Large Kernel Close + Endpoint 탐지 (Open path)

```python
# 두 줄 선을 하나로 병합
for ksize in [5, 30, 60, 80, 100]:
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksize, ksize))
    filled = cv2.morphologyEx(comp_mask, cv2.MORPH_CLOSE, k)
    skel   = skeletonize(filled > 0).astype(np.uint8)
    
    # Endpoint 탐지 (degree-1 픽셀)
    pts_set = set(zip(xs.tolist(), ys.tolist()))
    eps = [(x,y) for (x,y) in pts_set
           if sum(1 for dy in [-1,0,1] for dx in [-1,0,1]
                  if (dy,dx)!=(0,0) and (x+dx,y+dy) in pts_set) == 1]
    
    if len(eps) == 2:  # 깔끔한 단일 경로
        break
```

- Open path의 hollow 두 줄 선 → endpoint가 2개 이상으로 분기
- ksize를 순차적으로 키우며 endpoint가 정확히 2개가 될 때까지 시도
- Endpoint에서 시작하는 KDTree NN 정렬로 방향성 보장

---

## 6. 20px 등간격 선형 보간 샘플링

```python
def sample_by_dist(pts, step):
    result = [pts[0].copy()]
    acc = 0.0
    for i in range(1, len(pts)):
        d = np.linalg.norm(pts[i] - pts[i-1])
        acc += d
        while acc >= step:
            acc -= step
            r = (d - acc) / d if d > 0 else 0
            result.append(pts[i-1] + r * (pts[i] - pts[i-1]))  # 선형 보간
    return np.array(result)
```

- 단순 stride 샘플링이 아닌 누적 거리 기반 등간격 보간
- 곡선 구간에서도 정확히 20px 간격 유지
- 폐루프의 경우 마지막→첫 번째 gap은 자연스럽게 닫힘

---

## 7. CW/CCW 방향 감지 (Shoelace 공식)

```python
def detect_direction(wps):
    pts = np.array([[w['x'], w['y']] for w in wps])
    n = len(pts)
    area = sum(pts[i][0]*pts[(i+1)%n][1] - pts[(i+1)%n][0]*pts[i][1]
               for i in range(n))
    return "CW" if area > 0 else "CCW"
```

**좌표계 주의사항**  
이미지 좌표계는 y축이 아래 방향(↓)이므로 표준 수학 좌표계와 부호가 반전된다.

| Shoelace 부호 | 이미지(y↓) 화면 시각 방향 |
|---|---|
| area > 0 | CW (시계 방향) |
| area < 0 | CCW (반시계 방향) |

---

## 8. 경로 방향 설정

```python
DESIRED_DIR = {
    1:"CCW", 2:"CW",  3:"CCW", 4:"CCW", 5:"CW",
    6:"CW",  7:"CW",  8:"CW",  9:"CW",  10:"CW",
    11:"CCW",12:"CCW",13:"CCW",14:"CCW"
}
```

방향이 맞지 않으면 waypoint 배열을 역순으로 뒤집는다.

```python
def reverse_wps(wps):
    rev = wps[::-1]
    for i, w in enumerate(rev): w['id'] = i
    return rev
```

**Open path(R11~R14) 방향 주의**  
Open path는 Shoelace 공식이 적용되지 않으므로, 실제 화살표 방향을 시각적으로 확인한 뒤 역순 여부를 결정한다. 초반 waypoint들의 좌표 변화(Δx, Δy)로 진행 방향을 수동 검증했다.

---

## 9. JSON 스키마

### 개별 파일 (`waypoints_route{N}.json`)

```json
{
  "meta": {
    "route_id": 1,
    "route_name": "Outer Loop",
    "direction": "CCW",
    "closed": true,
    "total_waypoints": 368,
    "sample_step_px": 20,
    "dp_epsilon": 0,
    "resolution": "3840x2160",
    "coordinate_system": "image_px_origin_topleft_x_right_y_down"
  },
  "waypoints": [
    {"id": 0, "x": 1340.0, "y": 182.0},
    {"id": 1, "x": 1320.4, "y": 183.0}
  ]
}
```

### 통합 파일 (`route_map.json`)

```json
{
  "meta": {
    "description": "ARASEO-DALIMI mini-city track waypoints",
    "version": "1.0",
    "total_routes": 14,
    "total_waypoints": 2435,
    "resolution": "3840x2160",
    "sample_step_px": 20,
    "coordinate_system": "image_px_origin_topleft_x_right_y_down"
  },
  "routes": [
    {
      "route_id": 1,
      "route_name": "Outer Loop",
      "direction": "CCW",
      "closed": true,
      "total_waypoints": 368,
      "waypoints": [ ... ]
    }
  ]
}
```

---

## 10. 14개 경로 목록

| Route ID | 이름 | 유형 | 방향 | WP 수 | 비고 |
|---|---|---|---|---|---|
| R01 | Outer Loop | Closed | CCW | 368 | 트랙 외곽 전체 |
| R02 | Inner Loop | Closed | CW  | 293 | 내측 소루프 |
| R03 | Top-D      | Closed | CCW | 307 | 상단 D자형 |
| R04 | Bot-D      | Closed | CCW | 297 | 하단 D자형 |
| R05 | L-Rect     | Closed | CW  | 197 | 좌측 사각형 |
| R06 | R-Rect     | Closed | CW  | 190 | 우측 사각형 |
| R07 | TL Mini    | Closed | CW  | 123 | 좌상단 소형 |
| R08 | TR Mini    | Closed | CW  | 122 | 우상단 소형 |
| R09 | BL Mini    | Closed | CW  | 125 | 좌하단 소형 |
| R10 | BR Mini    | Closed | CW  | 117 | 우하단 소형 |
| R11 | Open J-SW  | Open   | CCW | 74  | 하단↑ → 좌측← |
| R12 | Open J-NE  | Open   | CCW | 71  | 우측← → 하단↓ |
| R13 | Open J-NW  | Open   | CCW | 72  | 상단↓ → 좌측← |
| R14 | Open J-SE  | Open   | CCW | 79  | 상단↓ → 우측→ |

**합계**: 2,435 waypoints

---

## 11. 트러블슈팅 이력

### 11-1. ratio ≈ 2.0 문제 (hollow skeleton)

**증상**: findContours로 얻은 contour 길이가 skeleton 픽셀 수의 약 2배  
**원인**: 얇은 선이 골격화되면 내외곽 두 줄 skeleton이 생성되어 dilate 후 ring 구조 형성  
**해결**: `len(contour) / skel_px > 1.7` 이면 KDTree NN 직접 정렬로 전환

### 11-2. Open path branch 문제

**증상**: ksize=5 skeleton에서 endpoint가 3개 이상 발생  
**원인**: hollow open path의 두 줄 선이 교차점 근처에서 branch 생성  
**해결**: ksize를 5→30→60→80→100 순서로 키우며 `len(eps) == 2`가 되는 최소 ksize 사용

### 11-3. 이미지 외곽 노이즈

**증상**: 이미지 경계부가 Magenta로 오인식, 가장 큰 컴포넌트로 선택됨  
**원인**: 촬영 환경 조명·반사로 이미지 테두리 픽셀이 HSV Magenta 범위에 포함  
**해결**: 이미지 border 30px를 0으로 마스킹, HSV saturation 하한을 80→100으로 강화

### 11-4. Rank1 컴포넌트가 노이즈

**증상**: 가장 큰 컴포넌트의 bbox가 이미지 전체에 걸침 (w=2662, h=2003)  
**해결**: `w > 2000 or h > 1500` 조건으로 Rank1 skip, 공간 필터(cx, cy, w, h)로 실제 경로 선택

### 11-5. Open path CW/CCW 판단 오류

**증상**: reversed=True로 설정해도 화면에서 화살표가 CW로 보임  
**원인**: 원본 추출 방향이 이미 CCW였는데 CCW를 만들기 위해 역전시켜 오히려 CW가 됨  
**교훈**: Open path는 Shoelace 공식 비적용 → 초반 5개 waypoint의 Δx, Δy로 실제 방향 육안 검증 필수

---

## 12. 파이프라인 적응형 분기 요약

```
ratio = len(contour) / skel_px

if ratio < 1.5:
    → findContours 방식 (단일 경로, 정상)

elif ratio ≥ 1.5 and RETR_CCOMP에서 inner hole 존재:
    → outer/inner 컨투어 평균 → 중심선

elif ratio ≥ 1.5 and inner hole 없음:
    → KDTree NN 직접 정렬

# Open path 추가 처리:
if endpoints > 2:
    → ksize를 키우며 MORPH_CLOSE 반복 → endpoints == 2 달성 후 NN 정렬
```

---

## 13. 검증 기준

| 항목 | 기준 | 검증 방법 |
|---|---|---|
| 샘플링 간격 | 20 px | 인접 WP 평균 거리 15~25 px |
| dp_epsilon | 0 | meta 필드 직접 확인 |
| 폐루프 gap | ≤ 25 px | WP[0] ↔ WP[-1] 거리 |
| 개별↔통합 일치 | 100% | WP[0], WP[-1] 좌표 비교 |
| 방향 | 지정값과 일치 | 시각화 화살표 육안 확인 |

---

## 14. 출력 파일 목록

```
waypoints_route1.json   ~ waypoints_route14.json   # 개별 (14개)
route_map.json                                      # 통합 (206.8 KB)
route_map_viewer.html                               # 인터랙티브 시각화 (293 KB)
all_routes_arrows.png                               # 방향 화살표 오버레이 이미지
```

### `route_map_viewer.html` 기능

- 경로별 Enable/Disable 토글 버튼
- ALL ON / ALL OFF 일괄 제어
- CCW / CW / OPEN 방향 배지 표시
- 활성 경로 수 & WP 수 실시간 카운트
- 창 크기 변경 시 캔버스 자동 리사이즈
- 트랙 이미지 + waypoints + 방향 화살촉 인라인 렌더링

---

## 15. Pure Pursuit 연동 시 주의사항

1. **좌표계 변환**: 이미지 좌표(y↓)를 로봇 좌표계(y↑)로 변환 시 `y_robot = H - y_px`
2. **픽셀→미터**: `px_per_meter` 캘리브레이션 값 필요 (이미지 내 기준 마커 활용)
3. **Closed loop 진입점**: WP[0]이 임의의 추출 시작점이므로, 로봇 현재 위치에서 가장 가까운 WP부터 추적 시작
4. **Open path 종료 처리**: WP[-1] 도달 시 다음 경로 전환 트리거 필요

---

*이 문서는 ARASEO-DALIMI 프로젝트의 waypoint 추출 세션 전체 기술 내용을 정리한 것이다.*
