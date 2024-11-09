package frc.robot.util;

import edu.wpi.first.math.util.Units;
import frc.robot.vision.VisionConstants.CameraConstants;

/** Classe para obter as alturas do sistema de visão */
public class Measures {

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
   * Retorna a posição X da câmera em metros pelo nome.
   *
   * @param camera nome da câmera usada
   * @return posição X da câmera em metros
   */
  public static double getCameraX(String camera) {
    if (camera.equals(CameraConstants.CAMERA_FL_NAME)) {
      return CameraConstants.CAMERA_FL_X;
    } else if (camera.equals(CameraConstants.CAMERA_FR_NAME)) {
      return CameraConstants.CAMERA_FR_X;
    } else if (camera.equals(CameraConstants.LIMELIGHT_NAME)) {
      return CameraConstants.LIMELIGHT_X;
    }
    return 0.0;
  }

  /**
   * Retorna a posição Y da câmera em metros pelo nome.
   *
   * @param camera nome da câmera usada
   * @return posição Y da câmera em metros
   */
  public static double getCameraY(String camera) {
    if (camera.equals(CameraConstants.CAMERA_FL_NAME)) {
      return CameraConstants.CAMERA_FL_Y;
    } else if (camera.equals(CameraConstants.CAMERA_FR_NAME)) {
      return CameraConstants.CAMERA_FR_Y;
    } else if (camera.equals(CameraConstants.LIMELIGHT_NAME)) {
      return CameraConstants.LIMELIGHT_Y;
    }
    return 0.0;
  }

  /**
   * Retorna a altura (Z) da câmera em metros pelo nome.
   *
   * @param camera nome da câmera usada
   * @return altura da câmera em metros
   */
  public static double getCameraZ(String camera) {
    if (camera.equals(CameraConstants.CAMERA_FL_NAME)) {
      return CameraConstants.CAMERA_FL_Z;
    } else if (camera.equals(CameraConstants.CAMERA_FR_NAME)) {
      return CameraConstants.CAMERA_FR_Z;
    } else if (camera.equals(CameraConstants.LIMELIGHT_NAME)) {
      return CameraConstants.LIMELIGHT_Z;
    }
    return 0.0;
  }

  /**
   * Retorna a rotação (Roll) da câmera em graus pelo nome.
   *
   * @param camera nome da câmera usada
   * @return rotação (Roll) da câmera em graus
   */
  public static double getCameraRoll(String camera) {
    if (camera.equals(CameraConstants.CAMERA_FL_NAME)) {
      return CameraConstants.CAMERA_FL_ROLL;
    } else if (camera.equals(CameraConstants.CAMERA_FR_NAME)) {
      return CameraConstants.CAMERA_FR_ROLL;
    } else if (camera.equals(CameraConstants.LIMELIGHT_NAME)) {
      return CameraConstants.LIMELIGHT_ROLL;
    }
    return 0.0;
  }

  /**
   * Retorna a inclinação (Pitch) da câmera em graus pelo nome.
   *
   * @param camera nome da câmera usada
   * @return inclinação (Pitch) da câmera em graus
   */
  public static double getCameraPitch(String camera) {
    if (camera.equals(CameraConstants.CAMERA_FL_NAME)) {
      return CameraConstants.CAMERA_FL_PITCH;
    } else if (camera.equals(CameraConstants.CAMERA_FR_NAME)) {
      return CameraConstants.CAMERA_FR_PITCH;
    } else if (camera.equals(CameraConstants.LIMELIGHT_NAME)) {
      return CameraConstants.LIMELIGHT_PITCH;
    }
    return 0.0;
  }

  /**
   * Retorna a rotação (Yaw) da câmera em graus pelo nome.
   *
   * @param camera nome da câmera usada
   * @return rotação (Yaw) da câmera em graus
   */
  public static double getCameraYaw(String camera) {
    if (camera.equals(CameraConstants.CAMERA_FL_NAME)) {
      return CameraConstants.CAMERA_FL_YAW;
    } else if (camera.equals(CameraConstants.CAMERA_FR_NAME)) {
      return CameraConstants.CAMERA_FR_YAW;
    } else if (camera.equals(CameraConstants.LIMELIGHT_NAME)) {
      return CameraConstants.LIMELIGHT_YAW;
    }
    return 0.0;
  }
}
