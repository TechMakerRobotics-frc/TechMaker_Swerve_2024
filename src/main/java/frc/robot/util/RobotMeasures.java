package frc.robot.util;

import edu.wpi.first.math.util.Units;
import frc.robot.util.UtilConstants.VisionConstants;

/** Classe para obter as alturas do sistema de visão */
public class RobotMeasures {

  private static double targetHeight;

  /**
   * Retorna a altura do alvo em metros com base no ID do AprilTag.
   *
   * @param tagId o ID da AprilTag
   * @return a altura do alvo em metros
   */
  public static double getTagHeight(int tagId) {
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

  /**
   * Retorna a altura da câmera em metros pelo nome.
   *
   * @param camera nome da câmera usada
   * @return altura câmera em metros
   */
  public static double getCameraHeight(String camera) {
    if (camera.equals(VisionConstants.CAMERA_FL_NAME)) {
      return VisionConstants.CAMERA_FL_HEIGHT;
    } else if (camera.equals(VisionConstants.CAMERA_FR_NAME)) {
      return VisionConstants.CAMERA_FR_HEIGHT;
    }
    return 0.0;
  }

  public static double getCameraAngle(String camera) {
    if (camera.equals(VisionConstants.CAMERA_FL_NAME)) {
      return VisionConstants.CAMERA_FL_ANGLE;
    } else if (camera.equals(VisionConstants.CAMERA_FR_NAME)) {
      return VisionConstants.CAMERA_FR_ANGLE;
    }
    return 0.0;
  }
}
