package frc.robot.commands;

public final class CommandConstants {

  public static final class AlignConstants {
    public static final double kvyStageP = 0.75;
    public static final double kvyStageI = 0.0;
    public static final double kvyStageD = 0.0;
    public static final double kDistanceFromSpeakerToShoot = 2.60;
    public static final double CAMERA_HEIGHT_METERS = 0.50;
    public static final double CAMERA_PITCH_RADIANS = Math.toRadians(30);

    public static final double LINEAR_P = 0.1;
    public static final double LINEAR_D = 0.0;

    public static final double ANGULAR_P = 0.1;
    public static final double ANGULAR_D = 0.0;

    public static final double kTargetArea = 1.20;

    public static final double kMaxPitch = -21;
  }
}