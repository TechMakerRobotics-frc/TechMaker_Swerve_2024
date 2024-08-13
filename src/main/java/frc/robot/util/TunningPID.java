package frc.robot.util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class TunningPID extends SubsystemBase {

  private static double p, i, d;
  private LoggedDashboardNumber kP = new LoggedDashboardNumber("PID/P", 0.01);

  private LoggedDashboardNumber kI = new LoggedDashboardNumber("PID/I", 0.01);

  private LoggedDashboardNumber kD = new LoggedDashboardNumber("PID/D", 0.01);

  public TunningPID() {}

  @Override
  public void periodic() {
    if (kP.get() != p) {
      System.out.println("P: " + kP.get());
      p = kP.get();
    }
    if (kI.get() != i) {
      System.out.println("I: " + kI.get());
      i = kI.get();
    }
    if (kD.get() != d) {
      System.out.println("D: " + kD.get());
      d = kD.get();
    }
  }

  public static double getP() {
    return p;
  }

  public static double getI() {
    return i;
  }

  public static double getD() {
    return d;
  }
}
