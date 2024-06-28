package frc.robot.util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class TunningPID extends SubsystemBase {

  private double p, i, d;
  private LoggedDashboardNumber kP = new LoggedDashboardNumber("PID/P", 1500.0);

  private LoggedDashboardNumber kI = new LoggedDashboardNumber("PID/I", 1500.0);

  private LoggedDashboardNumber kD = new LoggedDashboardNumber("PID/D", 1500.0);

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
}
