package frc.robot.vision;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

/**
 * Interface para o subsistema de visão, responsável por gerenciar e fornecer estimativas de pose do
 * robô a partir de dados de câmeras PhotonVision e AprilTags.
 */
public interface VisionPoseIO {

  /**
   * Atualiza a pose estimada global usando a câmera FL.
   *
   * @param prevEstimatedRobotPose A última pose estimada do robô.
   * @return Um {@link Optional} contendo a pose estimada do robô, se disponível.
   */
  Optional<EstimatedRobotPose> getEstimatedGlobalPoseFLCam();

  /**
   * Atualiza a pose estimada global usando a câmera FR.
   *
   * @param prevEstimatedRobotPose A última pose estimada do robô.
   * @return Um {@link Optional} contendo a pose estimada do robô, se disponível.
   */
  Optional<EstimatedRobotPose> getEstimatedGlobalPoseFRCam();

  /**
   * Atualiza a pose estimada global usando a câmera Limelight.
   *
   * @param prevEstimatedRobotPose A última pose estimada do robô.
   * @return Um {@link Optional} contendo a pose estimada do robô, se disponível.
   */
  Optional<EstimatedRobotPose> getEstimatedGlobalPoseLimelight();
}
