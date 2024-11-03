package frc.robot.util;

import org.apache.commons.math3.analysis.polynomials.PolynomialFunction;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

public class ThrottleMap {
    private final PolynomialSplineFunction throttleCurve;

    public ThrottleMap() {
        throttleCurve = createThrottleCurve();
    }

    private PolynomialSplineFunction createThrottleCurve() {
        double[] xValues = {0.0, 0.2, 0.5, 0.7, 1.0}; // Entrada (0 a 1)
        double[] yValues = {0.0, 0.1, 0.2, 0.5, 0.8}; // Saída desejada

        PolynomialFunction[] polynomials = new PolynomialFunction[xValues.length - 1];

        for (int i = 0; i < polynomials.length; i++) {
            // Coeficientes do polinômio interpolador entre (xValues[i], yValues[i]) e (xValues[i+1], yValues[i+1])
            double a = yValues[i];
            double b = (yValues[i + 1] - yValues[i]) / (xValues[i + 1] - xValues[i]);
            polynomials[i] = new PolynomialFunction(new double[]{a, b});
        }

        return new PolynomialSplineFunction(xValues, polynomials);
    }

    public double applyThrottle(double input) {
        return throttleCurve.value(Math.min(1.0, Math.max(0.0, input)));
    }
}
