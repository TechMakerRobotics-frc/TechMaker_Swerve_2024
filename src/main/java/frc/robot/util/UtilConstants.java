package frc.robot.util;

import edu.wpi.first.math.util.Units;

public final class UtilConstants {

  public static class VisionConstants {
    public static final String CAMERA_A_NAME = "SuperCam";
    public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(30);
    public static final double CAMERA_HEIGHT_METERS = 0.20;

    public static final int TARGET_7_X_POSITION_METERS = 3;
    public static final int TARGET_7_Y_POSITION_METERS = 3;
    public static final int TARGET_7_YAW_DEGREES = 0;

    public static final double CAMERA_POSITION_FROM_ROBOT_CENTER_X_METERS = 0.30;
    public static final double CAMERA_POSITION_FROM_ROBOT_CENTER_Y_METERS = 0.30;
  }
}
