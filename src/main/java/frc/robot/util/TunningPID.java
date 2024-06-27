package frc.robot.util;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TunningPID extends SubsystemBase {

  private double p;
  private double i;
  private double d;

  public TunningPID(){
    private final LoggedDashboardNumber kP =
    new LoggedDashboardNumber("tunning real - p", 1500.0);
    LoggedDashboardNumber("tunning real - p", 0.0);
    
    private final LoggedDashboardNumber kI =
    new LoggedDashboardNumber("tunning real - p", 1500.0);
    LoggedDashboardNumber("tunning real - i", 0.0);
    
    private final LoggedDashboardNumber kD =
    new LoggedDashboardNumber("tunning real - p", 1500.0);
    LoggedDashboardNumber("tunning real - d", 0.0);
  }
}

@Override
  public void periodic() {
    System.out.println("P: " + p);
    System.out.println("I: " + i);
    System.out.println("D: " + d);
  }
}
