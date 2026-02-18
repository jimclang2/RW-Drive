#ifndef CURVES_H
#define CURVES_H

// ============================================================================
// DRIVE CURVES MODULE
// Provides multiple joystick response curves for driver control.
// Change ACTIVE_CURVE below to switch between curve types for testing.
// ============================================================================

enum CurveType {
  CURVE_LINEAR,       // Direct 1:1 — no curve applied
  CURVE_EXPONENTIAL,  // Gentle low end, snappy high end (LemLib-style)
  CURVE_CUBIC,        // Popular in VEX — smooth and predictable
  CURVE_QUADRATIC,    // Softer than linear, less aggressive than cubic
  CURVE_SCURVE        // Sigmoid — smooth S-shape response
};

// ========================================================
// CHANGE THIS TO SELECT YOUR CURVE TYPE
// ========================================================
extern CurveType ACTIVE_CURVE;

// Shared parameters (apply to all curves)
extern double CURVE_DEADBAND;    // Joystick deadband (default: 3)
extern double CURVE_MIN_OUTPUT;  // Minimum motor output past deadband (default: 10)
extern double CURVE_STRENGTH;    // Curve intensity — meaning varies per curve type:
                                 //   EXPONENTIAL: exponent steepness (default: 1.05)
                                 //   CUBIC:       blend factor 0.0=linear, 1.0=full cubic (default: 0.8)
                                 //   QUADRATIC:   unused (pure quadratic)
                                 //   SCURVE:      sigmoid steepness (default: 0.05)
                                 //   LINEAR:      unused

// Main entry point — applies the active curve to a joystick input [-127, 127]
// Returns curved output in [-127, 127] range
double applyCurve(double input);

#endif // CURVES_H
