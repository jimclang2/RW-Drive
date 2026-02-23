#include "curves.h"
#include <cmath>
#include <algorithm>

// ============================================================================
// DEFAULT SETTINGS — Change these or ACTIVE_CURVE to tune your drive feel
// ============================================================================
CurveType ACTIVE_CURVE    = CURVE_EXPONENTIAL;
double CURVE_DEADBAND     = 3.0;
double CURVE_MIN_OUTPUT   = 10.0;
double CURVE_STRENGTH     = 1.05;
double CURVE_PARAM        = 0.0;   // Extra param for EXPONENTIAL/PIECEWISE (0 = use defaults)

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

// Squared — x² curve, smooth low-end, full power at max
// Same as Quadratic but named to match PROS convention
static double curveSquared(double input) {
  double sign = (input > 0) ? 1.0 : -1.0;
  double ax = fabs(input) / 127.0;
  return sign * ax * ax * 127.0;
}

// Piecewise Linear — two linear zones with a breakpoint
// param sets the breakpoint (default 0.3 = 30% stick → 15% output)
static double curvePiecewise(double input, double param) {
  double sign = (input > 0) ? 1.0 : -1.0;
  double ax = fabs(input) / 127.0;
  double bp = (param > 0.01) ? param : 0.3;   // breakpoint
  double low_out = 0.15;                        // output at breakpoint
  double result;
  if (ax <= bp) {
    result = (low_out / bp) * ax;
  } else {
    result = low_out + ((1.0 - low_out) / (1.0 - bp)) * (ax - bp);
  }
  return sign * result * 127.0;
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
    case CURVE_SQUARED:
      output = curveSquared(input);
      break;
    case CURVE_PIECEWISE:
      output = curvePiecewise(input, CURVE_PARAM);
      break;
  }

  // Enforce minimum output so motors actually move past deadband
  if (fabs(output) < CURVE_MIN_OUTPUT) {
    double sign = (input > 0) ? 1.0 : -1.0;
    output = CURVE_MIN_OUTPUT * sign;
  }

  return output;
}

// Human-readable names for the brain screen / controller display
std::string getCurveName(CurveType curve) {
  switch (curve) {
    case CURVE_LINEAR:      return "Linear";
    case CURVE_EXPONENTIAL: return "Exponential";
    case CURVE_CUBIC:       return "Cubic";
    case CURVE_QUADRATIC:   return "Quadratic";
    case CURVE_SCURVE:      return "S-Curve";
    case CURVE_SQUARED:     return "Squared";
    case CURVE_PIECEWISE:   return "Piecewise";
    default:                return "Unknown";
  }
}

// Cycle to the next curve (wraps back to LINEAR after PIECEWISE)
CurveType nextCurve(CurveType current) {
  int next = (static_cast<int>(current) + 1) % CURVE_TYPE_COUNT;
  return static_cast<CurveType>(next);
}
