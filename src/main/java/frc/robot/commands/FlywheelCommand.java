package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.CommandConstants.FlywheelConstants;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIOVictorSPX;
import frc.robot.subsystems.flywheel.LockWheel;

/**
 * Comando para controlar os flywheels (interno e externo) de um robô. Esta classe permite iniciar,
 * parar e ajustar a velocidade dos flywheels. Também permite controlar o flywheel externo com um
 * temporizador.
 */
public class FlywheelCommand extends Command {

  private Flywheel flywheel;
  private Flywheel lockWheel;
  private Timer timer;

  /**
   * Construtor da classe FlywheelCommand. Inicializa os subsistemas de flywheel e lockWheel, e o
   * temporizador.
   */
  public FlywheelCommand() {
    flywheel = new Flywheel(new FlywheelIOVictorSPX());
    lockWheel = new Flywheel(new LockWheel());
    timer = new Timer();
  }

  /**
   * Método para iniciar o flywheel interno com uma velocidade específica. Utilizado para ajustar a
   * velocidade negativa do flywheel interno.
   */
  public Command runInsideFlywheel(double velocity) {
    return new InstantCommand(() -> flywheel.runVelocity(-velocity));
  }

  /**
   * Método para iniciar o lockWheel interno. Define a velocidade do lockWheel para 1000 unidades.
   */
  public Command runInsideLockWheel(double velocity) {
    return new InstantCommand(() -> lockWheel.runVelocity(velocity));
  }

  /**
   * Método para iniciar o flywheel externo com controle de tempo. Inicia o temporizador e ajusta a
   * velocidade do flywheel externo. Se o tempo ultrapassar o valor especificado em
   * FlywheelConstants, inicia o lockWheel.
   */
  public Command runOutsideFlywheel(double velocityFlywheel, double velocityLockwheel) {
    return new InstantCommand(() -> OutsideFlywheel(velocityFlywheel, velocityLockwheel));
  }

  /**
   * Método para iniciar a flywheel e mantê-la ativada durante a partida, sem interrompe-la.
   *
   * @param velocity velocidade da flywheel
   * @return command que ativa flywheel
   */
  public Command runFlywheel(double velocity) {
    return new InstantCommand(() -> flywheel.runVelocity(velocity));
  }

  private void OutsideFlywheel(double velocityFlywheel, double velocityLockwheel) {
    timer.reset();
    timer.start();
    flywheel.runVelocity(velocityFlywheel);

    if (timer.get() > FlywheelConstants.TIME_TO_SHOOT) {
      lockWheel.runVelocity(velocityLockwheel);
    }
  }

  /**
   * Método para iniciar o lockWheel externo com uma velocidade negativa. Utilizado para ajustar a
   * direção oposta do lockWheel.
   */
  public Command runOutsideLockWheel(double velocity) {
    return new InstantCommand(() -> lockWheel.runVelocity(-velocity));
  }

  /** Método para parar ambos os flywheels. Interrompe a operação do flywheel e do lockWheel. */
  public Command stopFlywheels() {
    return new ParallelCommandGroup(
        new InstantCommand(() -> flywheel.stop()), new InstantCommand(() -> lockWheel.stop()));
  }

  /** Método para parar apenas o lockWheel. Interrompe a operação do lockWheel. */
  public Command stopLockWheel() {
    return new InstantCommand(() -> lockWheel.stop());
  }

  /**
   * Método chamado quando o comando termina ou é interrompido. Para ambos os flywheels e interrompe
   * o temporizador.
   *
   * @param interrupted indica se o comando foi interrompido manualmente.
   */
  @Override
  public void end(boolean interrupted) {
    stopFlywheels();
    timer.stop();
  }

  /**
   * Determina se o comando está completo.
   *
   * @return false para manter o comando em execução indefinidamente.
   */
  @Override
  public boolean isFinished() {
    // Retorne true se quiser finalizar o comando após certa condição
    return false;
  }
}
