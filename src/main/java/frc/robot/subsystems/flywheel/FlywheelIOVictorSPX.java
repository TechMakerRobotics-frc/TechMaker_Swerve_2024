// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.flywheel;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.math.util.Units;

public class FlywheelIOVictorSPX implements FlywheelIO {
  private static final double GEAR_RATIO = 1.5;

  private final WPI_VictorSPX leader = new WPI_VictorSPX(15);
  private final WPI_VictorSPX follower = new WPI_VictorSPX(16);

  private double currentVelocity = 0;
  private double appliedVolts = 0;
  private double leaderCurrent = 0;
  private double followerCurrent = 0;

  public FlywheelIOVictorSPX() {
    leader.configFactoryDefault();
    follower.configFactoryDefault();
    
    leader.setNeutralMode(NeutralMode.Coast);
    follower.setNeutralMode(NeutralMode.Coast);

    leader.configVoltageCompSaturation(12.0);
    leader.enableVoltageCompensation(true);
    follower.configVoltageCompSaturation(12.0);
    follower.enableVoltageCompensation(true);

    leader.configOpenloopRamp(0.1);  // Slope for ramping up voltage
    follower.follow(leader);         // Follower setup
  }

  /**
   * @param inputs
   */
  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    // VictorSPX doesn't provide direct sensor input for position/velocity,
    // so this is an approximation.
    inputs.positionRad = 0;  // Add encoder-based position reading if applicable
    inputs.velocityRadPerSec = Units.rotationsToRadians(currentVelocity) / GEAR_RATIO;
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = new double[] { leaderCurrent, followerCurrent };
  }

  @Override
  public void setVoltage(double volts) {
    leader.set(ControlMode.PercentOutput, volts / 12.0);  // Convert to percent output
    appliedVolts = volts;
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    // VictorSPX doesn't support velocity control directly without an external sensor, so we'll emulate with percent output.
    double targetRPM = Units.radiansToRotations(velocityRadPerSec) * 60.0;
    currentVelocity = targetRPM / GEAR_RATIO;
    leader.set(ControlMode.PercentOutput, ffVolts / 12.0);  // Open loop control
    appliedVolts = ffVolts;
  }

  @Override
  public void stop() {
    leader.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    // VictorSPX doesn't support onboard PID, this would need to be handled by external code (e.g., in the robot loop).
  }
}
