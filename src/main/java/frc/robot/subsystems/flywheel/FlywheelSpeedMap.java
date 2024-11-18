package frc.robot.subsystems.flywheel;

import org.apache.commons.math3.analysis.polynomials.PolynomialFunction;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

public class FlywheelSpeedMap {
  private final PolynomialSplineFunction speedCurve;

  public FlywheelSpeedMap() {
    speedCurve = createSpeedCurve();
  }

  private PolynomialSplineFunction createSpeedCurve() {
    // Valores de exemplo de distância (em metros) e velocidades da flywheel (em RPM)
    double[] distanceValues = {2.0, 3.0, 4.0, 5.0, 6.0}; // Distâncias de referência em metros
    double[] speedValues = {
      800.0, 1000.0, 2000.0, 2500.0, 3000.0
    }; // Velocidades correspondentes em RPM

    PolynomialFunction[] polynomials = new PolynomialFunction[distanceValues.length - 1];

    for (int i = 0; i < polynomials.length; i++) {
      // Calcula os coeficientes de cada segmento polinomial para interpolação
      double a = speedValues[i];
      double b =
          (speedValues[i + 1] - speedValues[i]) / (distanceValues[i + 1] - distanceValues[i]);
      polynomials[i] = new PolynomialFunction(new double[] {a, b});
    }

    return new PolynomialSplineFunction(distanceValues, polynomials);
  }

  public double CalculateFlywheelSpeed(double distance) {
    // Limita a distância entre o mínimo e máximo da tabela
    double clampedDistance = Math.min(Math.max(distance, 0.5), 3.0);
    return speedCurve.value(clampedDistance);
  }
}
