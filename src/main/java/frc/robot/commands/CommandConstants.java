package frc.robot.commands;

public final class CommandConstants {

  public static final class AlignConstants {
    public static final double VY_SPEAKER_P = 0.15;
    public static final double VY_SPEAKER_I = 0.0;
    public static final double VY_SPEAKER_D = 0.0;
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
