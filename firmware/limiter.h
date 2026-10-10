// Стерео лимитер с упреждением (lookahead), без перегрузок по сэмплам.
// Схема: требуемое усиление → экспоненциальный спад (release) → удержание минимума
// на N кадров → скользящее среднее на N кадров. Звук задержан на N-1 кадров, поэтому
// к моменту пика усиление уже гарантированно не выше требуемого, а переход плавный
// (атака = N кадров, без щелчков). Ниже потолка усиление ровно 1.0 — звук не трогается.
#pragma once
#include <math.h>
#include <stdint.h>

class Limiter {
 public:
  static constexpr int MAXN = 512;

  void setup(uint32_t rate, float threshDb, float lookaheadMs, float releaseMs) {
    int n = (int)lroundf(lookaheadMs * 0.001f * (float)rate);
    if (n < 1) n = 1;
    if (n > MAXN) n = MAXN;
    N = n;
    ceilAbs = 32767.f * powf(10.f, threshDb / 20.f);   // уровень, выше которого режем
    relCoef = expf(-1.f / (releaseMs * 0.001f * (float)rate));
    invNOne = 1.f / ((float)N * (float)ONE);
    reset();
  }

  void reset() {
    relState = 1.f;
    for (int i = 0; i < MAXN; i++) { boxR[i] = ONE; dL[i] = 0.f; dR[i] = 0.f; }
    sum = (uint32_t)N * ONE;
    boxPos = 0; dPos = 0; t = 0;
    qHead = qTail = 0;
  }

  // Один стерео-кадр. Возвращает применённое усиление (для индикации).
  inline float process(float inL, float inR, float& outL, float& outR) {
    float aL = inL < 0 ? -inL : inL, aR = inR < 0 ? -inR : inR;
    float pk = aL > aR ? aL : aR;
    float req = pk > ceilAbs ? ceilAbs / pk : 1.f;

    relState = 1.f + (relState - 1.f) * relCoef;           // release: возврат к 1.0
    if (req < relState) relState = req;

    // Скользящий минимум за последние N кадров (монотонная очередь)
    while (qTail != qHead && qVal[(qTail + QN - 1) % QN] >= relState) qTail = (qTail + QN - 1) % QN;
    qVal[qTail] = relState; qIdx[qTail] = t; qTail = (qTail + 1) % QN;
    while ((int32_t)(t - qIdx[qHead]) >= N) qHead = (qHead + 1) % QN;
    float held = qVal[qHead];

    // Скользящее среднее за N кадров — в целых Q20: без накопления ошибки,
    // округление вниз, поэтому усиление никогда не выше требуемого
    uint32_t hq = (uint32_t)(held * (float)ONE);
    sum += hq - boxR[boxPos];
    boxR[boxPos] = hq;
    if (++boxPos >= N) boxPos = 0;
    float g = (float)sum * invNOne;

    // Задержка звука на N-1 кадров
    dL[dPos] = inL; dR[dPos] = inR;
    int rd = dPos - (N - 1);
    if (rd < 0) rd += MAXN;
    outL = dL[rd] * g;
    outR = dR[rd] * g;
    if (++dPos >= MAXN) dPos = 0;
    t++;
    return g;
  }

  int lookaheadFrames() const { return N; }

 private:
  static constexpr int QN = MAXN + 2;
  static constexpr uint32_t ONE = 1u << 20;
  int N = 144;
  float ceilAbs = 27571.f, relCoef = 0.9998f, relState = 1.f, invNOne = 1.f / (144.f * ONE);
  uint32_t sum = 144u * ONE;
  uint32_t boxR[MAXN];
  float dL[MAXN], dR[MAXN];
  float qVal[QN];
  uint32_t qIdx[QN];
  int qHead = 0, qTail = 0, boxPos = 0, dPos = 0;
  uint32_t t = 0;
};
