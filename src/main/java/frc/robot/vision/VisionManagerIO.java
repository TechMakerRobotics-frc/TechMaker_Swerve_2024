package frc.robot.vision;

import edu.wpi.first.math.geometry.*;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.*;

/** Interface para gerenciamento de funcionalidades de visão. */
public interface VisionManagerIO {

  /**
   * Obtém o resultado mais recente do pipeline da câmera.
   *
   * @return O resultado mais recente do pipeline.
   */
  PhotonPipelineResult getLatestPipeline();

  /**
   * Obtém a instância da câmera utilizada.
   *
   * @return A câmera utilizada.
   */
  PhotonCamera getCamera();

  /**
   * Verifica se há algum alvo detectado no resultado do pipeline.
   *
   * @param result O resultado do pipeline.
   * @return {@code true} se houver alvos detectados, caso contrário {@code false}.
   */
  boolean hasTarget(PhotonPipelineResult result);

  /**
   * Obtém a lista de alvos detectados no resultado do pipeline.
   *
   * @param result O resultado do pipeline.
   * @return Lista de alvos detectados.
   */
  List<PhotonTrackedTarget> getTargets(PhotonPipelineResult result);

  /**
   * Obtém o melhor alvo detectado no resultado do pipeline.
   *
   * @param result O resultado do pipeline.
   * @return O melhor alvo detectado.
   */
  PhotonTrackedTarget getBestTarget(PhotonPipelineResult result);

  /**
   * Obtém o ângulo de guinada (Yaw) do alvo em graus.
   *
   * @param target O alvo utilizado.
   * @return O ângulo de guinada em graus.
   */
  double getYaw(PhotonTrackedTarget target);

  /**
   * Obtém o ângulo de inclinação (Pitch) do alvo em graus.
   *
   * @param target O alvo utilizado.
   * @return O ângulo de inclinação em graus.
   */
  double getPitch(PhotonTrackedTarget target);

  /**
   * Obtém a área ocupada pelo alvo no campo de visão da câmera como uma porcentagem.
   *
   * @param target O alvo utilizado.
   * @return A área ocupada pelo alvo como uma porcentagem.
   */
  double getArea(PhotonTrackedTarget target);

  /**
   * Obtém o valor de inclinação lateral (Skew) do alvo em graus.
   *
   * @param target O alvo utilizado.
   * @return O valor de inclinação lateral em graus.
   */
  double getSkew(PhotonTrackedTarget target);

  /**
   * Obtém os cantos delimitadores mínimos do alvo detectado.
   *
   * @param target O alvo utilizado.
   * @return Lista dos cantos delimitadores do alvo.
   */
  List<TargetCorner> getBoundingCorners(PhotonTrackedTarget target);

  /**
   * Obtém o identificador único do alvo detectado.
   *
   * @param target O alvo utilizado.
   * @return O identificador do alvo.
   */
  int getTargetId(PhotonTrackedTarget target);

  /**
   * Obtém a ambiguidade da pose calculada do alvo.
   *
   * @param target O alvo utilizado.
   * @return O valor da ambiguidade da pose.
   */
  double getPoseAbmiguity(PhotonTrackedTarget target);

  /**
   * Obtém a transformação com menor erro entre a câmera e o alvo detectado.
   *
   * @param target O alvo utilizado.
   * @return A transformação da câmera para o alvo.
   */
  Transform3d getBestCamera(PhotonTrackedTarget target);

  /**
   * Obtém uma transformação alternativa entre a câmera e o alvo detectado.
   *
   * @param target O alvo utilizado.
   * @return A transformação alternativa da câmera para o alvo.
   */
  Transform3d getAlternateCamera(PhotonTrackedTarget target);

  /**
   * Obtém o ângulo do melhor alvo detectado.
   *
   * @param target O alvo utilizado.
   * @return O ângulo do alvo em graus.
   */
  double getAngle(PhotonTrackedTarget target);

  /**
   * Obtém a distância entre a câmera e o alvo detectado.
   *
   * @param target O alvo utilizado.
   * @return A distância em metros.
   */
  double getDistance(PhotonTrackedTarget target);

  /**
   * Retorna a posição X da câmera em metros.
   *
   * @return A posição X da câmera.
   */
  double getCameraX();

  /**
   * Retorna a posição Y da câmera em metros.
   *
   * @return A posição Y da câmera.
   */
  double getCameraY();

  /**
   * Retorna a altura (Z) da câmera em metros.
   *
   * @return A altura (Z) da câmera.
   */
  double getCameraZ();

  /**
   * Retorna a rotação (Roll) da câmera em graus.
   *
   * @return A rotação (Roll) da câmera.
   */
  double getCameraRoll();

  /**
   * Retorna a inclinação (Pitch) da câmera em graus.
   *
   * @return A inclinação (Pitch) da câmera.
   */
  double getCameraPitch();

  /**
   * Retorna a rotação (Yaw) da câmera em graus.
   *
   * @return A rotação (Yaw) da câmera.
   */
  double getCameraYaw();

  /**
   * Retorna a posição da câmera como uma Translation3d.
   *
   * @return A posição da câmera.
   */
  Translation3d getCameraTranslation3d();

  /**
   * Retorna a rotação da câmera como uma Rotation3d.
   *
   * @return A rotação da câmera.
   */
  Rotation3d getCameraRotation3d();

  /**
   * Retorna a transformação completa da câmera (posição e rotação).
   *
   * @return A transformação da câmera.
   */
  Transform3d getCameraTransform3d();
}
