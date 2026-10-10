// Компрессор с ручками как у ReaComp (Cockos): Threshold, Ratio (0 = ∞), Knee, Attack,
// Release, Auto release, RMS size, Pre-comp (упреждение), Make-up. Стерео со связкой.
// Детектор — RMS по окну RMS size от max(L², R²); при RMS size = 0 — пиковый.
// Расчёт усиления — каждые CTRL кадров, между ними линейная интерполяция.
// Auto release: две огибающие — быстрая (заданный release) и медленная (×4 атака и release),
// берётся бóльшее ослабление. Короткие пики отпускаются быстро, долгое сжатие — плавно.
// Это классическая схема программно-зависимого release; алгоритм ReaComp закрыт, поведение близкое.
#pragma once
#include <math.h>
#include <stdint.h>

class ReaStyleComp {
 public:
  static constexpr int MAXRMS = 2048;   // до ~42 мс при 48 кГц
  static constexpr int MAXPRE = 1024;   // до ~21 мс упреждения
  static constexpr int CTRL = 4;

  void setup(uint32_t rate, float threshDb, float ratio, float kneeDb, float attackMs,
             float releaseMs, float rmsMs, float precompMs, bool autoRelease, float makeupDb) {
    fs = (float)rate;
    T = threshDb;
    invR = ratio <= 0.f ? 0.f : 1.f / ratio;          // 0 = бесконечность
    K = kneeDb < 0.f ? 0.f : kneeDb;
    autoRel = autoRelease;
    makeup = powf(10.f, makeupDb / 20.f);
    W = (int)lroundf(rmsMs * 0.001f * fs);
    if (W < 1) W = 1;
    if (W > MAXRMS) W = MAXRMS;
    P = (int)lroundf(precompMs * 0.001f * fs);
    if (P < 0) P = 0;
    if (P > MAXPRE - 1) P = MAXPRE - 1;
    auto coef = [&](float ms) { return 1.f - expf(-(float)CTRL / ((ms < 0.05f ? 0.05f : ms) * 0.001f * fs)); };
    aAtt = coef(attackMs);
    aRel = coef(releaseMs);
    aAttS = coef(attackMs * 4.f);
    aRelS = coef(releaseMs * 4.f);
    reset();
  }

  void reset() {
    for (int i = 0; i < MAXRMS; i++) pw[i] = 0.f;
    for (int i = 0; i < MAXPRE; i++) { dL[i] = 0.f; dR[i] = 0.f; }
    sum = 0.f; pPos = 0; dPos = 0; ctr = 0; sinceResum = 0;
    envF = envS = 0.f;
    gPrev = gNext = makeup; gStep = 0.f; gCur = makeup;
  }

  // Один стерео-кадр (значения в шкале int16). Возвращает текущее усиление (линейное, без make-up).
  inline float process(float inL, float inR, float& outL, float& outR) {
    // Детектор: скользящее среднее мощности
    float l = inL * (1.f / 32768.f), r = inR * (1.f / 32768.f);
    float p = l * l > r * r ? l * l : r * r;
    sum += p - pw[pPos];
    pw[pPos] = p;
    if (++pPos >= W) pPos = 0;
    if (++sinceResum >= 32768) {                        // сброс накопленной ошибки float
      sinceResum = 0; sum = 0.f;
      for (int i = 0; i < W; i++) sum += pw[i];
    }

    if (++ctr >= CTRL) {
      ctr = 0;
      float mean = sum / (float)W;
      float x = 10.f * log10f(mean > 1e-12f ? mean : 1e-12f);   // dBFS
      float gr = gainReduction(x);
      envF += (gr - envF) * (gr > envF ? aAtt : aRel);
      float g = envF;
      if (autoRel) {
        envS += (gr - envS) * (gr > envS ? aAttS : aRelS);
        if (envS > g) g = envS;
      }
      gPrev = gCur;
      gNext = powf(10.f, -g / 20.f) * makeup;
      gStep = (gNext - gPrev) / (float)CTRL;
    }
    gCur += gStep;
    if ((gStep > 0.f && gCur > gNext) || (gStep < 0.f && gCur < gNext)) gCur = gNext;

    // Упреждение (pre-comp): звук задерживается на P кадров
    dL[dPos] = inL; dR[dPos] = inR;
    int rd = dPos - P;
    if (rd < 0) rd += MAXPRE;
    outL = dL[rd] * gCur;
    outR = dR[rd] * gCur;
    if (++dPos >= MAXPRE) dPos = 0;
    return gCur / makeup;
  }

 private:
  float fs = 48000.f, T = -9.f, invR = 0.f, K = 0.f, makeup = 1.f;
  float aAtt = 0.f, aRel = 0.f, aAttS = 0.f, aRelS = 0.f;
  bool autoRel = true;
  int W = 240, P = 0;
  float pw[MAXRMS];
  float dL[MAXPRE], dR[MAXPRE];
  float sum = 0.f;
  int pPos = 0, dPos = 0, ctr = 0;
  uint32_t sinceResum = 0;
  float envF = 0.f, envS = 0.f;
  float gPrev = 1.f, gNext = 1.f, gStep = 0.f, gCur = 1.f;

  // Статическая кривая с мягким коленом: сколько dB убрать при уровне x
  inline float gainReduction(float x) const {
    float d = x - T;
    float y;
    if (K > 0.f && 2.f * fabsf(d) <= K) {
      float q = d + K * 0.5f;
      y = x + (invR - 1.f) * q * q / (2.f * K);
    } else if (d > 0.f) {
      y = T + d * invR;
    } else {
      y = x;
    }
    return x - y;
  }
};
