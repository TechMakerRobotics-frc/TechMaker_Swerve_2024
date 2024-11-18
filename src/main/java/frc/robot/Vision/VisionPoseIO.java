package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

/**
 * Interface para o subsistema de visão, responsável por gerenciar e fornecer estimativas
 * de pose do robô a partir de dados de câmeras PhotonVision e AprilTags.
 */
public interface VisionPoseIO {

    /**
     * Atualiza a pose estimada global usando a câmera FL.
     *
     * @param prevEstimatedRobotPose A última pose estimada do robô.
     * @return Um {@link Optional} contendo a pose estimada do robô, se disponível.
     */
    Optional<EstimatedRobotPose> getEstimatedGlobalPoseFLCam(Pose2d prevEstimatedRobotPose);

    /**
     * Atualiza a pose estimada global usando a câmera FR.
     *
     * @param prevEstimatedRobotPose A última pose estimada do robô.
     * @return Um {@link Optional} contendo a pose estimada do robô, se disponível.
     */
    Optional<EstimatedRobotPose> getEstimatedGlobalPoseFRCam(Pose2d prevEstimatedRobotPose);

    /**
     * Atualiza a pose estimada global usando a câmera Limelight.
     *
     * @param prevEstimatedRobotPose A última pose estimada do robô.
     * @return Um {@link Optional} contendo a pose estimada do robô, se disponível.
     */
    Optional<EstimatedRobotPose> getEstimatedGlobalPoseLimelight(Pose2d prevEstimatedRobotPose);

    /**
     * Obtém a pose 3D estimada pela câmera FL.
     *
     * @return A pose 3D estimada, se disponível; caso contrário, retorna {@code null}.
     */
    Pose3d getRobotPoseFLCam();

    /**
     * Obtém a pose 3D estimada pela câmera FR.
     *
     * @return A pose 3D estimada, se disponível; caso contrário, retorna {@code null}.
     */
    Pose3d getRobotPoseFRCam();

    /**
     * Obtém a pose 3D estimada pela câmera Limelight.
     *
     * @return A pose 3D estimada, se disponível; caso contrário, retorna {@code null}.
     */
    Pose3d getRobotPoseLimelight();
}
