#include "curves.h"
#include <cmath>

// ============================================================================
// DEFAULT SETTINGS — Change these or ACTIVE_CURVE to tune your drive feel
// ============================================================================
CurveType ACTIVE_CURVE    = CURVE_EXPONENTIAL;
double CURVE_DEADBAND     = 3.0;
double CURVE_MIN_OUTPUT   = 10.0;
double CURVE_STRENGTH     = 1.05;

// ============================================================================
// INDIVIDUAL CURVE IMPLEMENTATIONS
// All take raw input [-127, 127] and return shaped output [-127, 127]
// ============================================================================

// Linear — direct pass-through, no shaping
static double curveLinear(double input) {
  return input;
}

// Exponential — LemLib-style expo curve
// strength controls how aggressive the curve is (1.0 = nearly linear, 5.0 = very aggressive)
static double curveExponential(double input, double strength) {
  double sign = (input > 0) ? 1.0 : -1.0;
  return exp(strength * (fabs(input) - 127.0) / 127.0) * sign * 127.0;
}

// Cubic — most popular VEX curve
// strength blends between linear (0.0) and full cubic (1.0)
static double curveCubic(double input, double strength) {
  double normalized = input / 127.0;  // Normalize to [-1, 1]
  double cubic = normalized * normalized * normalized;  // x^3
  double blended = (1.0 - strength) * normalized + strength * cubic;
  return blended * 127.0;
}

// Quadratic — softer than cubic, good middle ground
static double curveQuadratic(double input) {
  double sign = (input > 0) ? 1.0 : -1.0;
  return (input * input / 127.0) * sign;
}

// S-Curve (Sigmoid) — smooth at both ends, responsive in the middle
// strength controls steepness of the sigmoid (default: 0.05)
static double curveSCurve(double input, double strength) {
  double output = 127.0 * (2.0 / (1.0 + exp(-strength * input)) - 1.0);
  return output;
}

// ============================================================================
// MAIN ENTRY POINT
// Applies deadband, selected curve, and minimum output enforcement
// ============================================================================
double applyCurve(double input) {
  // Deadband — ignore tiny stick movements
  if (fabs(input) <= CURVE_DEADBAND) return 0.0;

  // Apply the selected curve
  double output = 0.0;
  switch (ACTIVE_CURVE) {
    case CURVE_LINEAR:
      output = curveLinear(input);
      break;
    case CURVE_EXPONENTIAL:
      output = curveExponential(input, CURVE_STRENGTH);
      break;
    case CURVE_CUBIC:
      output = curveCubic(input, CURVE_STRENGTH);
      break;
    case CURVE_QUADRATIC:
      output = curveQuadratic(input);
      break;
    case CURVE_SCURVE:
      output = curveSCurve(input, CURVE_STRENGTH);
      break;
  }

  // Enforce minimum output so motors actually move past deadband
  if (fabs(output) < CURVE_MIN_OUTPUT) {
    double sign = (input > 0) ? 1.0 : -1.0;
    output = CURVE_MIN_OUTPUT * sign;
  }

  return output;
}
