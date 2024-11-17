package frc.robot.util;

public final class UtilConstants {

  public static class VisionConstants {
    public static final String CAMERA_FL_NAME = "FLCam";
    public static final double CAMERA_FL_HEIGHT = 0.22;
    public static final double CAMERA_FL_ANGLE = 25;

    public static final String CAMERA_FR_NAME = "FRCam";
    public static final double CAMERA_FR_HEIGHT = 0.22;
    public static final double CAMERA_FR_ANGLE = 25;

    public static final String LIMELIGHT_NAME = "Limelight";
    public static final double LIMELIGHT_HEIGHT = 0.45;
    public static final double LIMELIGHT_ANGLE = 20;

    public static final int TARGET_7_X_POSITION_METERS = 3;
    public static final int TARGET_7_Y_POSITION_METERS = 3;
    public static final int TARGET_7_YAW_DEGREES = 0;

    public static final double CAMERA_POSITION_FROM_ROBOT_CENTER_X_METERS = 0.30;
    public static final double CAMERA_POSITION_FROM_ROBOT_CENTER_Y_METERS = 0.30;
  }

  public final class ThrottleConstants {
    public static final double[] X_VALUES = {0.0, 0.2, 0.5, 0.7, 1.0}; // Entrada (0 a 1)
    public static final double[] Y_VALUES = {0.0, 0.1, 0.2, 0.3, 0.8}; // Saída desejada
  } 
}
