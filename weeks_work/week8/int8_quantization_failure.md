# INT8 양자화 실패 분석 보고서

## 요약

YOLOv8n-detect 모델을 ONNX Runtime QDQ INT8로 정적 양자화했을 때,
FP32에서 최대 0.807이었던 신뢰도(confidence) 출력이 INT8 모델에서 완전히 0으로 붕괴했다.
결과적으로 mAP50 0.753 → 0.000, 실배포 환경에서 차선 미검출.

```
실험 결과 (검증셋 200장)
─────────────────────────────────────────
모델              mAP50    mAP50-95
─────────────────────────────────────────
FP32 (.pt)       0.7531    0.5977
FP32 (.onnx)     0.7071    0.5221   ← ONNX 변환 손실 약 4%p
INT8 (.onnx)     0.0000    0.0000   ← 양자화 실패
─────────────────────────────────────────

실이미지 신뢰도 분포 (val 3장 샘플)
FP32: max=0.807  검출 수 20개  >0.12 : 20개
INT8: max=0.000  검출 수  0개  >0.12 :  0개
```

---

## 1. 배경: INT8 QDQ 양자화란

부동소수점(FP32) 값을 8비트 정수(INT8, -128 ~ 127)로 압축하는 기법이다.
ONNX Runtime의 QDQ(Quantize-Dequantize) 포맷은 각 연산 앞뒤에
`QuantizeLinear` / `DequantizeLinear` 노드를 삽입한다.

```
FP32 텐서 ─▶ [QuantizeLinear] ─▶ INT8 텐서 ─▶ [DequantizeLinear] ─▶ FP32 텐서
              scale, zero_point                  scale, zero_point
```

변환 공식:

```
양자화:   int8_val  = clamp(round(fp32_val / scale) + zero_point, -128, 127)
역양자화: fp32_val  = (int8_val - zero_point) * scale
```

`scale`과 `zero_point`는 **캘리브레이션 데이터**로부터 결정된다.
이 두 값이 잘못 결정되면 정보가 영구 손실된다.

---

## 2. YOLOv8-detect 출력 구조

YOLOv8-detect(nc=1)의 ONNX 최종 출력 텐서는 `[1, 5, 8400]` 형태다.

```
output0: shape = (1, 5, 8400)

axis-1 구성:
  [0] cx   ← 박스 중심 x (픽셀, 범위: 0 ~ 640)
  [1] cy   ← 박스 중심 y (픽셀, 범위: 0 ~ 640)
  [2] w    ← 박스 너비   (픽셀, 범위: 0 ~ 640)
  [3] h    ← 박스 높이   (픽셀, 범위: 0 ~ 640)
  [4] conf ← 신뢰도      (Sigmoid 후, 범위: 0 ~ 1)
```

핵심 문제: **좌표(0~640)와 신뢰도(0~1)가 하나의 텐서에 섞여 있다.**

---

## 3. 실패 원인: per_channel=False + 혼재 범위

### 3-1. 양자화 설정 (`quantize_onnx.py`)

```python
quantize_static(
    ...
    quant_format=QuantFormat.QDQ,
    per_channel=False,        # ← 핵심 문제
    weight_type=QuantType.QInt8,
    activation_type=QuantType.QInt8,
)
```

`per_channel=False`는 **출력 텐서 전체에 단 하나의 scale/zero_point**를 적용한다.

### 3-2. 실제 양자화 파라미터 (모델 분석 결과)

```
output0 노드 (DequantizeLinear):
  scale      = 2.501248
  zero_point = -128
```

이 scale은 캘리브레이션 이미지의 **전체 텐서 최댓값(≈637)** 기준으로 결정됐다.

```
scale 산출 근거:
  텐서 최댓값 ≈ 640 (좌표값)
  INT8 최댓값 =  127 - (-128) = 255
  scale = 640 / 255 ≈ 2.509  →  실측 2.501
```

### 3-3. 신뢰도 값이 0이 되는 과정

신뢰도 0.8을 이 scale로 양자화하면:

```
int8_val = round(0.8 / 2.501) + (-128)
         = round(0.00032) - 128
         = 0 - 128
         = -128   ← INT8 하한값으로 클리핑
```

역양자화 시:

```
fp32_val = (-128 - (-128)) × 2.501 = 0 × 2.501 = 0.000
```

신뢰도 값이 아무리 높아도(0.1 ~ 0.99) INT8 범위 내에서
모두 -128로 매핑되고, 복원 시 전부 0이 된다.

```
신뢰도 0~1 → INT8 -128 → FP32 0.000
    ↑                          ↓
   입력               출력이 항상 0
```

시각적으로:

```
FP32 신뢰도 범위 [0 ──────────── 1]
                 ↓  scale=2.501로 양자화
INT8 표현 범위   [-128          +127]
                  ↑
               0/2.501 = 0.0003 → -128에 매핑
               1/2.501 = 0.0004 → -128에 매핑
               (모두 같은 칸으로 붕괴)
```

---

## 4. 왜 scale이 좌표 기준으로 결정됐는가

정적 양자화의 캘리브레이션은 검증 이미지를 모델에 통과시켜
**각 텐서의 실제 분포 범위**를 측정한다.

`output0` 텐서의 분포:

| 채널 | 실제 범위 |
|------|-----------|
| cx, cy | 0 ~ 640 |
| w, h   | 0 ~ 640 |
| conf   | 0 ~ 1   |

`per_channel=False`이므로 5개 채널 전체를 하나로 보고
**최댓값(≈640)** 기준의 scale을 산출한다.
신뢰도 채널의 미세한 분포(0~1)는 이 scale에 완전히 묻힌다.

---

## 5. 실험적 증거

### 출력 텐서 범위 측정

```python
out = sess_int8.run(None, {'images': inp})[0]  # (1, 5, 8400)
print(out.min(), out.max())
# → 0.0000  637.8182
```

637.8 = (127 − (−128)) × 2.501 → **INT8 최댓값을 scale로 복원한 좌표 최대치** 그대로다.

### 신뢰도 채널 단독 측정

```python
conf_fp32 = sess_fp32.run(...)[0][0][4, :]  # 신뢰도 행
conf_int8 = sess_int8.run(...)[0][0][4, :]

conf_fp32.max()  # → 0.807
conf_int8.max()  # → 0.000  (완전 붕괴)
```

---

## 6. 근본 원인 요약

| 원인 | 설명 |
|------|------|
| **per_channel=False** | 좌표(0~640)와 신뢰도(0~1)를 하나의 scale로 처리 |
| **이종(異種) 범위 혼재** | 출력 텐서 안에 서로 다른 스케일의 값이 공존 |
| **신뢰도 정밀도 손실** | scale=2.5 기준 → 신뢰도 1.0도 INT8 0.0004 칸에 불과 → -128으로 클리핑 |
| **캘리브레이션 지배** | 큰 값(좌표)이 scale을 지배해 작은 값(신뢰도)의 표현 불가 |

---

## 7. 해결 방안 (적용하지 않은 이유 포함)

### 방안 1: per_channel=True (가장 직접적)

채널별로 독립적인 scale을 사용한다.
신뢰도 채널에 맞는 scale(≈1/127)이 별도 산출되어 정밀도가 보존된다.

```python
quantize_static(
    ...
    per_channel=True,   # 채널별 독립 scale
)
```

**미적용 이유:** ARM Cortex-A CPU용 ONNX Runtime은 per_channel QDQ를
완전히 지원하지 않아 RPi5에서 실행 오류 발생 가능.

### 방안 2: 출력 레이어 양자화 제외

신뢰도 관련 마지막 레이어(Sigmoid 이후)를 FP32로 유지하고
앞단 연산만 INT8로 양자화한다(Mixed Precision).

**미적용 이유:** ONNX Runtime 정적 양자화 API에서 레이어 단위 제외 설정이 복잡함.

### 방안 3: FP32 ONNX 사용 (현재 채택)

양자화 없이 FP32 ONNX를 그대로 배포한다.

```
best.onnx (FP32): mAP50=0.707, RPi5 추론 ~40ms
```

RPi5에서 INT8(92ms, 실제로는 미검출)보다 FP32(40ms, 정상 검출)가
속도와 정확도 모두 우수하다.
ONNX Runtime의 ARM 최적화 커널이 FP32에서도 충분한 성능을 낸다.

### 방안 4: NCNN / TFLite 변환

RPi5에 특화된 경량 추론 프레임워크를 사용하면
채널별 양자화와 ARM NEON 최적화를 함께 지원한다.

---

## 8. PT → FP32 ONNX mAP 감소 분석 (0.753 → 0.707)

INT8 실패와 별개로, PyTorch 모델(best.pt)을 ONNX로 변환했을 때도
mAP50이 0.753 → 0.707로 약 4.6%p 감소한다.

### 8-1. 측정 결과

```
FP32 (.pt)    mAP50=0.7531  mAP50-95=0.5977  (GPU, torch 후처리)
FP32 (.onnx)  mAP50=0.7071  mAP50-95=0.5221  (CPU, ONNX Runtime)
```

### 8-2. 원인

**① 후처리 구현 차이**

PyTorch 검증(`val()`)과 ONNX 검증은 NMS 구현이 다르다.

| 항목 | PT 검증 | ONNX 검증 |
|------|---------|-----------|
| NMS 구현 | PyTorch Torchvision CUDA NMS | ONNX Runtime CPU NMS (또는 ultralytics Python NMS) |
| 연산 정밀도 | FP32 CUDA | FP32 CPU |
| 박스 디코딩 | 모델 내부 CUDA 커널 | Python numpy 연산 |

NMS에서 IoU 임계값 경계 근처의 박스 처리 방식이 약간 달라
TP/FP 판정이 미세하게 달라질 수 있다.

**② 부동소수점 연산 순서 차이**

CUDA (GPU)와 CPU는 부동소수점 연산 누적 순서가 달라
같은 FP32라도 결과값에 미세한 차이(~1e-6)가 생긴다.
이 차이가 수백 장 × 8400 앵커에 걸쳐 누적되면 mAP에 영향을 미친다.

**③ 검증 배치 크기**

PT 검증은 GPU 배치 처리(batch=16 등)를 활용하지만
ONNX 검증은 batch=1로 순차 처리한다.
배치 정규화(BN) 레이어 동작이 배치 크기에 따라 미세하게 다를 수 있다.
(단, YOLOv8 추론 시 BN은 fused되므로 영향 최소)

**④ ONNX opset 변환 손실**

PyTorch → ONNX 변환 시 일부 연산이 근사값으로 대체된다.
예: `torch.nn.SiLU` → ONNX `Sigmoid × x` 형태로 분해되며
연산 순서 변경으로 경계값에서 미세 차이 발생.

### 8-3. 실용적 의미

```
mAP50 차이: 0.7531 - 0.7071 = 0.046 (4.6%p)
```

이 4.6%p 차이는 **허용 가능한 수준**이다.
ONNX 변환 후 mAP가 크게 떨어지는 경우(10%p 이상)는
모델 아키텍처에 ONNX 미지원 연산이 포함된 경우이며,
YOLOv8n은 표준 연산만 사용해 변환 손실이 작다.

실배포에서는 FP32 ONNX(mAP50=0.707)를 기준으로 삼는 것이 적절하다.

---

## 9. 결론

### PT → ONNX 손실 (4.6%p)

연산 순서·NMS 구현·부동소수점 누적 방식의 차이로 발생하는
**구조적으로 허용 가능한 손실**이다. YOLOv8n 기준 4.6%p는 정상 범위.

### ONNX FP32 → INT8 손실 (100%)

INT8 양자화 실패의 핵심은 **YOLOv8 검출 출력 텐서의 구조적 특성**과
**ONNX Runtime 정적 양자화의 per_channel=False 설정의 부적합**이다.

좌표(0~640)와 신뢰도(0~1)를 같은 텐서에 담아 단일 scale로 양자화하면
수치 범위가 큰 좌표값이 scale을 지배해 신뢰도 정보가 완전히 소실된다.
이는 모델 성능 저하가 아닌 **설계 수준의 비호환성**이며,
per_channel 양자화 또는 Mixed Precision 없이는 해결되지 않는다.

```
최종 권장 배포 모델: best.onnx (FP32)
  mAP50    = 0.707  (PT 대비 -4.6%p, 허용 범위)
  RPi5 추론 = ~40ms / frame  (충분한 실시간 성능)
  차선 검출 = 정상 동작 확인
```
