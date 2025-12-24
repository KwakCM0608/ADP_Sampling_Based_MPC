# 🚗 Autonomous Driving Algorithm Performance Evaluation

본 프로젝트는 자율주행 차량 제어 알고리즘 **Pure Pursuit (PP)**, **Hybrid**, **MPC**
세 가지 방식을 동일한 주행 로그 기반으로 **정량 평가 및 비교**하는 것을 목표로 한다.

---

## 1. Evaluated Algorithms

* **Pure Pursuit (PP)**

  * 기하학 기반 경로 추종 알고리즘
* **Hybrid**

  * Pure Pursuit 기반에 보정 로직을 결합한 방식
* **Model Predictive Control (MPC)**

  * 미래 상태를 예측하여 최적 제어 입력을 계산하는 모델 기반 제어

모든 알고리즘은 동일한 트랙 조건에서 **8랩 주행 로그**를 기반으로 평가하였다.

---

## 2. Log Description

각 알고리즘은 다음 형식의 `.txt` 로그 파일을 생성한다.

| 변수명                 | 설명                          |
| ------------------- | --------------------------- |
| `x_m`, `y_m`        | 차량 위치 (map 좌표계)             |
| `track_error`       | 기준 경로로부터의 횡방향 거리 (m)        |
| `heading_error_rad` | 차량 헤딩 − 경로 헤딩 (rad, −π ~ π) |
| `speed_mps`         | 실제 차량 속도                    |
| `speed_cmd_mps`     | 제어기가 출력한 속도 명령              |
| `steer_cmd_rad`     | 제어기가 출력한 조향 명령              |

> 속도 및 조향 명령 관련 항목은 본 평가에서는 사용하지 않는다.

---

## 3. Evaluation Metrics

총 **5개 평가 항목**을 사용하였다.

### 3.1 Track Error (경로 추종 성능)

1. **Track Error Mean**
   [
   \overline{e_{\text{track}}}
   = \frac{1}{N}\sum_{i=1}^{N} e_{\text{track},i}
   ]

* 전체 주행 동안의 평균 경로 오차
* 작을수록 경로를 정확히 추종

2. **Track Error 95% (95th percentile)**
   [
   e_{\text{track},95}
   = \text{Quantile}*{0.95}(e*{\text{track}})
   ]

* 큰 오차 구간(tail behavior)을 반영
* 코너, 불안정 상황에서의 성능을 평가

---

### 3.2 Heading Error (자세 정렬 성능)

모든 heading error는 **절대값 → degree 변환** 후 계산한다.

[
e_{\text{head},i}^{\circ}
= |\text{heading_error_rad}_i| \cdot \frac{180}{\pi}
]

3. **Heading Error Mean**
   [
   \overline{e_{\text{head}}^{\circ}}
   = \frac{1}{N}\sum_{i=1}^{N} e_{\text{head},i}^{\circ}
   ]

4. **Heading Error 95%**
   [
   e_{\text{head},95}^{\circ}
   = \text{Quantile}*{0.95}(e*{\text{head}}^{\circ})
   ]

---

### 3.3 Lap Time (주행 효율)

* 로그는 각 알고리즘마다 **8랩 주행 기준**으로 기록됨
* 제어 루프 주기: **40 Hz**

[
T_{\text{lap}} = \frac{N}{40 \times 8}
]

* 1랩 평균 소요 시간 (초)
* 작을수록 빠른 주행

---

## 4. Scoring Method (Percentage-Based)

각 항목별로 **가장 성능이 좋은 알고리즘을 100%**로 두고,
나머지는 선형적으로 점수를 환산하였다.

[
\text{Score(%)} = 100 \cdot \frac{\text{Best Value}}{\text{Algorithm Value}}
]

> 모든 항목은 **작을수록 좋은 지표**이다.

---

## 5. Per-Metric Performance (%)

| Algorithm  |  Track Mean |   Track 95% | Heading Mean | Heading 95% |    Lap Time |
| ---------- | ----------: | ----------: | -----------: | ----------: | ----------: |
| **MPC**    |      93.14% | **100.00%** |  **100.00%** | **100.00%** | **100.00%** |
| **PP**     |      96.58% |      72.83% |       92.59% |      89.97% |      97.34% |
| **Hybrid** | **100.00%** |      65.59% |       87.69% |      87.43% |      96.70% |

---

## 6. Total Performance Score (Normalized)

총점은 5개 항목(각각 100%)을 합산한 뒤, **500% 기준으로 정규화**하였다.

[
\text{Total Performance (%)} =
\frac{\sum \text{Metric Scores}}{500} \times 100
]

| Algorithm  | Total Score (%) |
| ---------- | --------------: |
| **MPC**    |      **98.63%** |
| **PP**     |          89.86% |
| **Hybrid** |          87.48% |

---

## 7. Visualization

* **Radar Chart (Per-Metric % Performance)**
  → 각 평가 항목별 상대 성능 비교
* **Normalized Total Performance (%)**
  → 알고리즘 간 종합 성능 비교

> Radar chart는 각 항목의 성능 편차 및 알고리즘 특성을 직관적으로 보여준다.

---

## 8. Conclusion

* **MPC**

  * 대부분 항목에서 최고 성능
  * 빠른 랩타임과 낮은 heading/track tail error
  * **종합 성능 1위**

* **Pure Pursuit (PP)**

  * 모든 항목에서 안정적인 중상위 성능
  * 극단적인 강점은 없으나 균형 잡힌 베이스라인

* **Hybrid**

  * 평균 track error는 가장 작음
  * 그러나 큰 오차 구간(95%)에서 성능 저하
  * **평균은 좋으나 한계 상황에서 취약**

---

## 9. Summary

> **MPC는 안정성과 속도를 동시에 만족하는 가장 우수한 알고리즘이며,
> PP는 안정적인 기준선 역할, Hybrid는 평균 성능은 뛰어나지만 tail 성능 개선이 필요하다.**

---
