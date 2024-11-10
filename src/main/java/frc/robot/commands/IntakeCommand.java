package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSparkMax;

/**
 * Comando para controlar os Intakes (interno e externo) de um robô. Esta classe permite iniciar,
 * parar e ajustar a velocidade dos Intakes. Também permite controlar o Intake externo com um
 * temporizador.
 */
public class IntakeCommand extends Command {

  private Intake Intake;

  /**
   * Construtor da classe IntakeCommand. Inicializa o subsistema de Intake.
   */
  public IntakeCommand() {
    Intake = new Intake(new IntakeIOSparkMax());
  }

  /**
   * Método para iniciar o Intake com uma velocidade específica para pegar o elemento.
   */
  public Command runInsideIntake(double velocity) {
    return new InstantCommand(() -> Intake.runVelocity(-velocity));
  }

  /**
   * Método para iniciar o Intake com uma velocidade específica para tirar o elemento.
   */
  public Command runOutsideIntake(double velocity) {
    return new InstantCommand(() -> Intake.runVelocity(velocity));
  }

  public Command stopIntake() {
    return new InstantCommand(() -> Intake.stop());
  }

  /**
   * Método chamado quando o comando termina ou é interrompido.
   *
   * @param interrupted indica se o comando foi interrompido manualmente.
   */
  @Override
  public void end(boolean interrupted) {
    stopIntake();
  }

  /**
   * Determina se o comando está completo.
   *
   * @return false para manter o comando em execução indefinidamente.
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}
