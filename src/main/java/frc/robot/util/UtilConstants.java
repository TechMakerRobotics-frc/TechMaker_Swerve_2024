package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class UtilConstants {

  public static class VisionConstants {
    public static final String CAMERA_FL_NAME = "FLCam";
    public static final double CAMERA_FL_X = 0.265;
    public static final double CAMERA_FL_Y = 0.235;
    public static final double CAMERA_FL_Z = 0.20;
    public static final double CAMERA_FL_ROLL = 0.0;
    public static final double CAMERA_FL_PITCH = 30;
    public static final double CAMERA_FL_YAW = 25;
    
    public static final String CAMERA_FR_NAME = "FRCam";
    public static final double CAMERA_FR_X = 0.265;
    public static final double CAMERA_FR_Y = -0.235;
    public static final double CAMERA_FR_Z = 0.20;
    public static final double CAMERA_FR_ROLL = 0.0;
    public static final double CAMERA_FR_PITCH = 30;
    public static final double CAMERA_FR_YAW = -25;

    public static final String LIMELIGHT_NAME = "Limelight";
    public static final double LIMELIGHT_X = 0.155;
    public static final double LIMELIGHT_Y = 0;
    public static final double LIMELIGHT_Z = 0.545;
    public static final double LIMELIGHT_ROLL = 180;
    public static final double LIMELIGHT_PITCH = 25;
    public static final double LIMELIGHT_YAW = 180;

    public static final Transform3d ROBOT_TO_FL_CAM =
      new Transform3d(
          new Translation3d(CAMERA_FL_X, CAMERA_FL_Y, CAMERA_FL_Z),
          new Rotation3d(CAMERA_FL_ROLL, CAMERA_FL_PITCH, CAMERA_FL_YAW)
      );

    public static final Transform3d ROBOT_TO_FR_CAM =
      new Transform3d(
          new Translation3d(CAMERA_FR_X, CAMERA_FR_Y, CAMERA_FR_Z),
          new Rotation3d(CAMERA_FR_ROLL, CAMERA_FR_PITCH, CAMERA_FR_YAW)
      );

    public static final Transform3d ROBOT_TO_LIMELIGHT =
      new Transform3d(
          new Translation3d(LIMELIGHT_X, LIMELIGHT_Y, LIMELIGHT_Z),
          new Rotation3d(LIMELIGHT_ROLL, LIMELIGHT_PITCH, LIMELIGHT_YAW)
      );

  }
}
