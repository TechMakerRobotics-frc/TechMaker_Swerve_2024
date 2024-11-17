package frc.robot.vision;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.vision.VisionConstants.CameraConstants;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.*;

/** Gerencia as funcionalidades relacionadas à visão computacional utilizando a PhotonVision. */
public class VisionManager {

  private final PhotonCamera camera;

  /**
   * Construtor para inicializar o gerenciador de visão com a câmera especificada.
   *
   * @param camera Instância da câmera PhotonVision a ser utilizada.
   */
  public VisionManager(PhotonCamera camera) {
    this.camera = camera;
  }

  /**
   * Obtém o resultado mais recente do pipeline da câmera.
   *
   * @return O resultado mais recente do pipeline da câmera.
   */
  public PhotonPipelineResult getLatestPipeline() {
    return camera.getLatestResult();
  }

  /**
   * Obtém a instância da câmera utilizada.
   *
   * @return A câmera utilizada.
   */
  public PhotonCamera getCamera() {
    return camera;
  }

  /**
   * Verifica se há algum alvo detectado no resultado do pipeline.
   *
   * @param result O resultado do pipeline.
   * @return {@code true} se houver alvos detectados; {@code false} caso contrário.
   */
  public boolean hasTarget(PhotonPipelineResult result) {
    return result != null ? result.hasTargets() : false;
  }

  /**
   * Obtém a lista de alvos detectados no resultado do pipeline.
   *
   * @param result O resultado do pipeline.
   * @return Lista de alvos detectados.
   */
  public List<PhotonTrackedTarget> getTargets(PhotonPipelineResult result) {
    return result.getTargets();
  }

  /**
   * Obtém o melhor alvo detectado no resultado do pipeline.
   *
   * @param result O resultado do pipeline.
   * @return O melhor alvo detectado.
   */
  public PhotonTrackedTarget getBestTarget(PhotonPipelineResult result) {
    return result.getBestTarget();
  }

  /**
   * Obtém o ângulo de guinada (Yaw) do alvo em graus.
   *
   * @param target O alvo utilizado.
   * @return O ângulo de guinada em graus (positivo para a direita).
   */
  public double getYaw(PhotonTrackedTarget target) {
    return target.getYaw();
  }

  /**
   * Obtém o ângulo de inclinação (Pitch) do alvo em graus.
   *
   * @param target O alvo utilizado.
   * @return O ângulo de inclinação em graus (positivo para cima).
   */
  public double getPitch(PhotonTrackedTarget target) {
    return target.getPitch();
  }

  /**
   * Obtém a área ocupada pelo alvo no campo de visão da câmera como uma porcentagem.
   *
   * @param target O alvo utilizado.
   * @return A área ocupada pelo alvo como uma porcentagem (0-100).
   */
  public double getArea(PhotonTrackedTarget target) {
    return target.getArea();
  }

  /**
   * Obtém o valor de inclinação lateral (Skew) do alvo em graus.
   *
   * @param target O alvo utilizado.
   * @return O valor de inclinação lateral em graus (positivo no sentido anti-horário).
   */
  public double getSkew(PhotonTrackedTarget target) {
    return target.getSkew();
  }

  /**
   * Obtém os cantos delimitadores mínimos do alvo detectado.
   *
   * @param target O alvo utilizado.
   * @return Lista dos cantos delimitadores do alvo.
   */
  public List<TargetCorner> getBoundingCorners(PhotonTrackedTarget target) {
    return target.getDetectedCorners();
  }

  /**
   * Obtém o identificador único do alvo detectado.
   *
   * @param target O alvo utilizado.
   * @return O identificador do alvo.
   */
  public int getTargetId(PhotonTrackedTarget target) {
    return target.getFiducialId();
  }

  /**
   * Obtém a ambiguidade da pose calculada do alvo.
   *
   * @param target O alvo utilizado.
   * @return O valor da ambiguidade da pose.
   */
  public double getPoseAbmiguity(PhotonTrackedTarget target) {
    return target.getPoseAmbiguity();
  }

  /**
   * Obtém a transformação com menor erro entre a câmera e o alvo detectado.
   *
   * @param target O alvo utilizado.
   * @return A transformação da câmera para o alvo.
   */
  public Transform3d getBestCamera(PhotonTrackedTarget target) {
    return target.getBestCameraToTarget();
  }

  /**
   * Obtém uma transformação alternativa entre a câmera e o alvo detectado.
   *
   * @param target O alvo utilizado.
   * @return A transformação alternativa da câmera para o alvo.
   */
  public Transform3d getAlternateCamera(PhotonTrackedTarget target) {
    return target.getAlternateCameraToTarget();
  }

  /**
   * Obtém o ângulo do melhor alvo detectado.
   *
   * @param target O alvo utilizado.
   * @return O ângulo do alvo em graus.
   */
  public double getAngle(PhotonTrackedTarget target) {
    return getBestCamera(target).getRotation().toRotation2d().getDegrees();
  }

  /**
   * Obtém a distância entre a câmera e o alvo detectado.
   *
   * @param target O alvo utilizado.
   * @return A distância em metros.
   */
  public double getDistance(PhotonTrackedTarget target) {
    if (getBestCamera(target) != null) {
      return getBestCamera(target).getX();
    } else {
      return 0.0;
    }
  }

  /**
   * Retorna a posição X da câmera em metros.
   *
   * @return posição X da câmera em metros
   */
  public double getCameraX() {
    if (this.camera.getName().equals(CameraConstants.CAMERA_FL_NAME)) {
      return CameraConstants.CAMERA_FL_X;
    } else if (this.camera.getName().equals(CameraConstants.CAMERA_FR_NAME)) {
      return CameraConstants.CAMERA_FR_X;
    } else if (this.camera.getName().equals(CameraConstants.LIMELIGHT_NAME)) {
      return CameraConstants.LIMELIGHT_X;
    }
    return 0.0;
  }

  /**
   * Retorna a posição Y da câmera em metros.
   *
   * @return posição Y da câmera em metros
   */
  public double getCameraY() {
    if (this.camera.getName().equals(CameraConstants.CAMERA_FL_NAME)) {
      return CameraConstants.CAMERA_FL_Y;
    } else if (this.camera.getName().equals(CameraConstants.CAMERA_FR_NAME)) {
      return CameraConstants.CAMERA_FR_Y;
    } else if (this.camera.getName().equals(CameraConstants.LIMELIGHT_NAME)) {
      return CameraConstants.LIMELIGHT_Y;
    }
    return 0.0;
  }

  /**
   * Retorna a altura (Z) da câmera em metros.
   *
   * @return altura da câmera em metros
   */
  public double getCameraZ() {
    if (this.camera.getName().equals(CameraConstants.CAMERA_FL_NAME)) {
      return CameraConstants.CAMERA_FL_Z;
    } else if (this.camera.getName().equals(CameraConstants.CAMERA_FR_NAME)) {
      return CameraConstants.CAMERA_FR_Z;
    } else if (this.camera.getName().equals(CameraConstants.LIMELIGHT_NAME)) {
      return CameraConstants.LIMELIGHT_Z;
    }
    return 0.0;
  }

  /**
   * Retorna a rotação (Roll) da câmera em graus.
   *
   * @return rotação (Roll) da câmera em graus
   */
  public double getCameraRoll() {
    if (this.camera.getName().equals(CameraConstants.CAMERA_FL_NAME)) {
      return CameraConstants.CAMERA_FL_ROLL;
    } else if (this.camera.getName().equals(CameraConstants.CAMERA_FR_NAME)) {
      return CameraConstants.CAMERA_FR_ROLL;
    } else if (this.camera.getName().equals(CameraConstants.LIMELIGHT_NAME)) {
      return CameraConstants.LIMELIGHT_ROLL;
    }
    return 0.0;
  }

  /**
   * Retorna a inclinação (Pitch) da câmera em graus.
   *
   * @return inclinação (Pitch) da câmera em graus
   */
  public double getCameraPitch() {
    if (this.camera.getName().equals(CameraConstants.CAMERA_FL_NAME)) {
      return CameraConstants.CAMERA_FL_PITCH;
    } else if (this.camera.getName().equals(CameraConstants.CAMERA_FR_NAME)) {
      return CameraConstants.CAMERA_FR_PITCH;
    } else if (this.camera.getName().equals(CameraConstants.LIMELIGHT_NAME)) {
      return CameraConstants.LIMELIGHT_PITCH;
    }
    return 0.0;
  }

  /**
   * Retorna a rotação (Yaw) da câmera em graus.
   *
   * @return rotação (Yaw) da câmera em graus
   */
  public double getCameraYaw() {
    if (this.camera.getName().equals(CameraConstants.CAMERA_FL_NAME)) {
      return CameraConstants.CAMERA_FL_YAW;
    } else if (this.camera.getName().equals(CameraConstants.CAMERA_FR_NAME)) {
      return CameraConstants.CAMERA_FR_YAW;
    } else if (this.camera.getName().equals(CameraConstants.LIMELIGHT_NAME)) {
      return CameraConstants.LIMELIGHT_YAW;
    }
    return 0.0;
  }
}
