package frc.robot.util;

import edu.wpi.first.math.util.Units;

/** 
 * Classe para obter a altura do alvo com base no ID do AprilTag. 
 */
public class IdTargetHeight {

  private static double targetHeight;

  /**
   * Retorna a altura do alvo em metros com base no ID do AprilTag.
   *
   * @param tagId o ID da AprilTag
   * @return a altura do alvo em metros
   */
  public static double idToHeight(int tagId) {
    switch (tagId) {
      case 1, 2, 5, 6, 9, 10 -> targetHeight = Units.inchesToMeters(53.38);
      case 3, 4, 7, 8 -> targetHeight = Units.inchesToMeters(57.13);
      case 11, 12, 13, 14, 15, 16 -> targetHeight = Units.inchesToMeters(52.00);
      default -> {
        System.out.println("ID para AprilTag inválida");
        targetHeight = 0.0;
      }
    }
    return targetHeight;
  }
}
