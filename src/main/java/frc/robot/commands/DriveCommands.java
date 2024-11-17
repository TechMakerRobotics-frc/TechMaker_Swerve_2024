package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.ThrottleMap;
import java.util.function.DoubleSupplier;

public class DriveCommands {
  private static final double DEADBAND = 0.15;

  private DriveCommands() {}

  /**
   * @param drive
   * @param xSupplier
   * @param ySupplier
   * @param omegaSupplier
   * @return Command
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {

    ThrottleMap throttleMap = new ThrottleMap();

    return Commands.run(
        () -> {
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          linearMagnitude = throttleMap.applyThrottle(linearMagnitude);
          omega = throttleMap.applyThrottleAbs(omega);

          // Square values for finer control at low speeds
          linearMagnitude = Math.copySign(linearMagnitude * linearMagnitude, linearMagnitude);
          omega = Math.copySign(omega * omega, omega);

          // Calculate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(
                      new Transform2d(new Translation2d(linearMagnitude, 0.0), new Rotation2d()))
                  .getTranslation();

          // Convert to field relative speeds & send command
          boolean isFlipped =
              DriverStation.getAlliance().map(alliance -> alliance == Alliance.Red).orElse(false);

          Rotation2d fieldRelativeRotation =
              isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation();

          ChassisSpeeds fieldRelativeSpeeds =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec(),
                  fieldRelativeRotation);

          drive.runVelocity(fieldRelativeSpeeds);
        },
        drive);
  }
}
