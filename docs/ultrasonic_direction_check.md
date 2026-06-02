# 초음파 3센서 통합 + 방향 배치 점검 가이드

> 하드웨어팀용. SW 작성자가 아니어도 이 문서만 보고 끝까지 할 수 있게 작성.
> 도구: `ultrasonic_direction_check.py` (실행하면 같은 안내가 화면에도 나옵니다)

## 이게 뭘 확인하나
1. **통합**: 3개 초음파(앞/왼/오)가 실제 주행 코드(`hal/ultrasonics.py`)를 통해
   동시에 정상 측정되는지.
2. **방향 배치**: "왼쪽"이라고 단 센서가 진짜 왼쪽을 보는지 등. 한쪽에만 장애물을
   두고 **세 센서를 동시에 읽어, 기대한 센서가 실제로 잡는지** 교차검증합니다.
   → 기존 `hardware_check.py`로는 못 잡는 **좌우 바뀜**까지 잡아냅니다.

## 준비물
- 평평한 판이나 책, 손바닥 (반사면). 모터·바퀴는 안 씁니다.
- 라즈베리파이 + 조립된 차. 자/책상 위에서 해도 됩니다.

## 실행
```
cd ~/teamkim-bml1
git pull
python ultrasonic_direction_check.py
```
(하드웨어 없이 흐름만 미리 보려면: `python ultrasonic_direction_check.py --demo`)

## 순서 (화면 안내대로 Enter)
1. **[0] 기준선** — 세 방향 다 비우고 Enter. 세 센서가 모두 먼 값(예: 80cm+)으로
   나오면 통합 OK. `--`(신호 없음)가 보이면 그 센서는 죽은 것.
2. **[FRONT]** — 차 **정면 바로 앞 10~15cm** 에 판을 대고 Enter.
3. **[LEFT]** — **전방 약 45° 왼쪽** 10~15cm 에 대고 Enter. (다른 쪽은 비우기)
4. **[RIGHT]** — **전방 약 45° 오른쪽** 10~15cm 에 대고 Enter.

> 핵심: 매 단계 **딱 한 방향에만** 장애물. 나머지는 50cm 이상 비워두세요.

## 결과 읽는 법
| 표시 | 뜻 | 조치 |
|---|---|---|
| ✅ **PASS** | 기대한 센서가 단독으로 가깝게 잡음 | 정상 |
| ❌ **WRONG_SENSOR** | 다른 센서가 더 가깝게 잡힘 (특히 좌우) | **배선/장착 바뀜** — 아래 참고 |
| ❌ **DEAD_EXPECTED** | 기대 센서가 신호 없음(`None`) | 죽음/배선 — `docs/hardware_troubleshooting.md` |
| ⚠️ **NO_DETECTION** | 아무 센서도 가깝게 못 잡음 | 장애물을 더 가까이(≤25cm) 다시 |
| ⚠️ **AMBIGUOUS** | 기대 센서가 제일 가깝지만 옆 센서도 가까움 | 장애물을 더 한쪽으로 치우쳐 다시 |

마지막에 **요약**이 나옵니다. 셋 다 PASS면 `✅ 통합 OK + 방향 배치 정확`.

## FAIL일 때
- **좌우 바뀜 (WRONG_SENSOR, 좌↔우)**: LEFT45(TRIG 25 / ECHO 8) 와
  RIGHT45(TRIG 7 / ECHO 12) 의 **배선이 서로 바뀌었거나 센서 장착 위치가 바뀐** 것.
  핀 연결 또는 물리적 장착을 서로 맞바꿔 고친 뒤 다시 실행.
- **신호 없음 (DEAD_EXPECTED)**: 해당 센서의 VCC(5V)/GND/TRIG/ECHO 배선,
  점퍼 접촉 확인. `docs/hardware_troubleshooting.md` 의 LEFT45/RIGHT45 절 참고.
- 고친 뒤에는 **셋 다 PASS** 나올 때까지 반복.

## 배선 기준 (BCM)
| 센서 | TRIG | ECHO |
|---|---|---|
| FRONT | 23 | 24 |
| LEFT45 | 25 | 8 |
| RIGHT45 | 7 | 12 |

측정 결과는 `logs/runs/<시각>_ultrasonic-direction-check.jsonl` 에 자동 기록되므로,
끝나면 그 파일을 SW 담당에게 넘기면 됩니다.
