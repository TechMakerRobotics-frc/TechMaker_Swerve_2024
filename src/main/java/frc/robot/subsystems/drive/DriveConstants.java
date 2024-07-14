package frc.robot.subsystems.drive;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

public final class DriveConstants {
    private static final double DRIVE_THROTTLE_INPUT_CURVE_X[] = {
      0.0, 0.100, 0.200, 0.300, 0.400, 0.500, 0.600, 0.700, 0.800, 0.900, 1.000
    };
    private static final double DRIVE_THROTTLE_INPUT_CURVE_Y[] = {
      0.0, 0.052, 0.207, 0.465, 0.827, 1.293, 1.862, 2.534, 3.310, 4.189, 5.172
    };
    private static final double DRIVE_TURN_INPUT_CURVE_X[] = {
      0.0, 0.100, 0.200, 0.300, 0.400, 0.500, 0.600, 0.700, 0.800, 0.900, 1.0
    };
    private static final double DRIVE_TURN_INPUT_CURVE_Y[] = {
      0.0, 0.010, 0.050, 0.100, 0.150, 0.200, 0.250, 0.300, 0.400, 0.600, 1.0
    };

    private static final SplineInterpolator SPLINE_INTERPOLATOR = new SplineInterpolator();
    public static final PolynomialSplineFunction DRIVE_THROTTLE_INPUT_CURVE =
        SPLINE_INTERPOLATOR.interpolate(DRIVE_THROTTLE_INPUT_CURVE_X, DRIVE_THROTTLE_INPUT_CURVE_Y);
    public static final PolynomialSplineFunction DRIVE_TURN_INPUT_CURVE =
        SPLINE_INTERPOLATOR.interpolate(DRIVE_TURN_INPUT_CURVE_X, DRIVE_TURN_INPUT_CURVE_Y);
  }