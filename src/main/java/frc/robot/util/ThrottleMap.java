package frc.robot.util;

import frc.robot.util.UtilConstants.ThrottleConstants;
import org.apache.commons.math3.analysis.polynomials.PolynomialFunction;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

/**
 * Classe que implementa um mapeamento de throttle utilizando uma curva de spline. Permite ajustar a
 * resposta dos controles para entrada e saída mais suaves ou personalizadas.
 */
public class ThrottleMap {
  private final PolynomialSplineFunction throttleCurve;

  /**
   * Construtor da classe ThrottleMap. Inicializa a curva de throttle com os valores predefinidos.
   */
  public ThrottleMap() {
    throttleCurve = createThrottleCurve();
  }

  /**
   * Cria uma curva de spline para mapear entradas de throttle para saídas ajustadas.
   *
   * @return Um objeto {@link PolynomialSplineFunction} que representa a curva de throttle.
   */
  private PolynomialSplineFunction createThrottleCurve() {
    double[] xValues = ThrottleConstants.X_VALUES;
    double[] yValues = ThrottleConstants.Y_VALUES;

    PolynomialFunction[] polynomials = new PolynomialFunction[xValues.length - 1];

    for (int i = 0; i < polynomials.length; i++) {
      // Coeficientes do polinômio interpolador entre (xValues[i], yValues[i]) e (xValues[i+1],
      // yValues[i+1])
      double a = yValues[i];
      double b = (yValues[i + 1] - yValues[i]) / (xValues[i + 1] - xValues[i]);
      polynomials[i] = new PolynomialFunction(new double[] {a, b});
    }

    return new PolynomialSplineFunction(xValues, polynomials);
  }

  /**
   * Aplica o mapeamento de throttle na entrada fornecida, limitando-a entre 0 e 1.
   *
   * @param input O valor de entrada (normalmente entre 0 e 1).
   * @return O valor ajustado pela curva de throttle.
   */
  public double applyThrottle(double input) {
    return throttleCurve.value(Math.min(1.0, Math.max(0.0, input)));
  }

  /**
   * Aplica o mapeamento de throttle na entrada fornecida, preservando o sinal original do valor de
   * entrada.
   *
   * @param input O valor de entrada, que pode ser positivo ou negativo.
   * @return O valor ajustado pela curva de throttle, mantendo o mesmo sinal do valor de entrada.
   */
  public double applyThrottleAbs(double input) {
    double magnitude = Math.min(1.0, Math.max(0.0, Math.abs(input)));
    return Math.copySign(throttleCurve.value(magnitude), input);
  }
}
