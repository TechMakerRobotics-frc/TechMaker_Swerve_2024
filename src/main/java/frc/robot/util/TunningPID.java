package frc.robot.util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
 * A classe TunningPID é responsável por gerenciar e ajustar em tempo real
 * os valores de controle PID (Proporcional, Integral e Derivativo). Os valores
 * são obtidos diretamente do AdvantageScope e podem ser atualizados conforme necessário
 * durante o funcionamento do robô. 
 */
public class TunningPID extends SubsystemBase {

    /** Instância Singleton da classe TunningPID */
    private static TunningPID instance;

    /** Valor proporcional do PID */
    private double p;
    /** Valor integral do PID */
    private double i;
    /** Valor derivativo do PID */
    private double d;

    /** Dashboard para ajustar o valor proporcional P */
    private LoggedDashboardNumber kP = new LoggedDashboardNumber("PID/P", 0.01);
    /** Dashboard para ajustar o valor integral I */
    private LoggedDashboardNumber kI = new LoggedDashboardNumber("PID/I", 0.01);
    /** Dashboard para ajustar o valor derivativo D */
    private LoggedDashboardNumber kD = new LoggedDashboardNumber("PID/D", 0.01);

    /**
     * Construtor da classe TunningPID. Inicializa os valores de P, I e D
     * com os valores obtidos do AdvantageScope.
     */
    public TunningPID() {
        this.p = kP.get();
        this.i = kI.get();
        this.d = kD.get();
    }

    /**
     * Método chamado periodicamente para atualizar os valores PID.
     * Se houver alterações nos valores de P, I ou D fornecidos pelo AdvantageScope,
     * eles são atualizados no sistema.
     */
    @Override
    public void periodic() {
        if (kP.get() != p) {
            p = kP.get();
        }
        if (kI.get() != i) {
            i = kI.get();
        }
        if (kD.get() != d) {
            d = kD.get();
        }
    }

    /**
     * Retorna o valor proporcional P atualizado.
     * 
     * @return Valor atual de P
     */
    public double getP() {
        return p;
    }

    /**
     * Retorna o valor integral I atualizado.
     * 
     * @return Valor atual de I
     */
    public double getI() {
        return i;
    }

    /**
     * Retorna o valor derivativo D atualizado.
     * 
     * @return Valor atual de D
     */
    public double getD() {
        return d;
    }

    /**
     * Retorna a instância Singleton da classe TunningPID. Se ainda não houver
     * uma instância, uma nova será criada.
     * 
     * @return Instância única da classe TunningPID
     */
    public static TunningPID getInstance() {
        if (instance == null) {
            instance = new TunningPID();
        }
        return instance;
    }
}
