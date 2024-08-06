package frc.robot.commands;

public final class CommandConstants {

  public static final class AlignConstants {
    public static final double VY_SPEAKER_P = 0.8;
    public static final double VY_SPEAKER_I = 0.0;
    public static final double VY_SPEAKER_D = 0.0;

    public static final double VX_SPEAKER_P = 0.05;
    public static final double VX_SPEAKER_I = 0.0;
    public static final double VX_SPEAKER_D = 0.0;

    public static final double VOMEGA_SPEAKER_P = 0.02;
    public static final double VOMEGA_SPEAKER_I = 0.0;
    public static final double VOMEGA_SPEAKER_D = 0.0;

    public static final double kDistanceFromSpeakerToShoot = 2.60;
    public static final double CAMERA_HEIGHT_METERS = 0.50;
    public static final double CAMERA_PITCH_RADIANS = Math.toRadians(30);

    public static final double LINEAR_P = 0.1;
    public static final double LINEAR_D = 0.0;

    public static final double ANGULAR_P = 0.1;
    public static final double ANGULAR_D = 0.0;

    public static final double kTargetArea = 1.20;

    public static final double kMaxPitch = -21;

    public static final double kvyStageP = 0.75;
    public static final double kvyStageI = 0.0;
    public static final double kvyStageD = 0.0;

    public static final double kvyAmpP = 0.08;
    public static final double kvyAmpI = 0.0;
    public static final double kvyAmpD = 0.0;

    public static final double kvyNoteP = 0.03;
    public static final double kvyNoteI = 0.002;
    public static final double kvyNoteD = 0.0;
  }

  public static class LimelightConstants {
    public static final int kIDSpeakerBlue = 7;
    public static final int kIDSpeakerRed = 3;

    public static final int kBlueSpeakerPipeline = 7;
    public static final int kRedSpeakerPipeline = 7;

    public static final int kPosePipeline = 2;

    public static final double kLimelightHeight = 0.2;
    public static final double kLimelightPanningAngle = 30;
  }

  public static class VisionConstants {
    public static final int kIDAmpBlue = 6;
    public static final int kIDAmpRed = 5;

    public static final int kIDSpeakerBlue = 7;
    public static final int kIDSpeakerRed = 7;

    public static final double kPhotonvisionHeight = 0.2;
    public static final double kPhotonvisionPanningAngle = 30;
  }
}
