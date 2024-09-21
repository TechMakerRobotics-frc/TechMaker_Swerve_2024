package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class ModuleIOSparkAndTalon implements ModuleIO {
  // Gear ratios for SDS MK4i L3, adjust as necessary
  private static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
  private static final double TURN_GEAR_RATIO = 150.0 / 7.0;

  private final CANSparkMax turnSparkMax;
  private final RelativeEncoder turnRelativeEncoder;
  private final CANcoder cancoder;
  private final StatusSignal<Double> turnAbsolutePosition;
  private final TalonFX driveTalon;

  private boolean isTurnMotorInverted = true;
  private final Rotation2d absoluteEncoderOffset;

  private final StatusSignal<Double> drivePosition;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveCurrent;

  public ModuleIOSparkAndTalon(int index) {
    switch (index) {
      case 0: // front left
        driveTalon = new TalonFX(8);
        turnSparkMax = new CANSparkMax(11, MotorType.kBrushless);
        cancoder = new CANcoder(2);
        absoluteEncoderOffset = new Rotation2d(Units.degreesToRadians(312.891)); // MUST BE CALIBRATED
        break;
      case 1: // front right
        driveTalon = new TalonFX(6);
        turnSparkMax = new CANSparkMax(4, MotorType.kBrushless);
        cancoder = new CANcoder(5);
        absoluteEncoderOffset = new Rotation2d(Units.degreesToRadians(123.135)); // MUST BE CALIBRATED
        break;
      case 2: // back left
        driveTalon = new TalonFX(12);
        turnSparkMax = new CANSparkMax(10, MotorType.kBrushless);
        cancoder = new CANcoder(1);
        absoluteEncoderOffset = new Rotation2d(Units.degreesToRadians(-60.381)); // MUST BE CALIBRATED
        break;
      case 3: // back right
        driveTalon = new TalonFX(9);
        turnSparkMax = new CANSparkMax(7, MotorType.kBrushless);
        cancoder = new CANcoder(3);
        absoluteEncoderOffset = new Rotation2d(Units.degreesToRadians(228.779)); // MUST BE CALIBRATED
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    // Initialize TalonFX configuration
    var driveConfig = new TalonFXConfiguration();
    driveConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveTalon.getConfigurator().apply(driveConfig);
    setDriveBrakeMode(false);

    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getSupplyCurrent();

    //Ver sobre essa frequência *******************************************************************************************************
    BaseStatusSignal.setUpdateFrequencyForAll(100.0, drivePosition); // Required for odometry
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, driveVelocity, driveAppliedVolts, driveCurrent);

    // Initialize turn motor (CANSparkMax)
    turnSparkMax.restoreFactoryDefaults();
    //Ver sobre esse Timeout *******************************************************************************************************
    turnSparkMax.setCANTimeout(250);
    turnRelativeEncoder = turnSparkMax.getEncoder();
    turnSparkMax.setInverted(isTurnMotorInverted);
    turnSparkMax.setSmartCurrentLimit(30);
    turnSparkMax.enableVoltageCompensation(10.0);
    turnRelativeEncoder.setPosition(0.0);
    turnRelativeEncoder.setMeasurementPeriod(10);
    turnRelativeEncoder.setAverageDepth(2);
    turnSparkMax.setCANTimeout(0);
    //Ver se isso diminui vida útil da memória ***************************************************************************************
    turnSparkMax.burnFlash();

    // Initialize CANcoder for turn position
    var cancoderConfig = new CANcoderConfiguration();
    cancoder.getConfigurator().apply(cancoderConfig);
    turnAbsolutePosition = cancoder.getAbsolutePosition();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Update drive (TalonFX)
    BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent);
    inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble()) / DRIVE_GEAR_RATIO;
    inputs.driveVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(driveVelocity.getValueAsDouble()) / DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = new double[] {driveCurrent.getValueAsDouble()};

    // Update turn (CANSparkMax)
    inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
        .minus(absoluteEncoderOffset);
    inputs.turnPosition = Rotation2d.fromRotations(turnRelativeEncoder.getPosition() / TURN_GEAR_RATIO);
    inputs.turnVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity()) / TURN_GEAR_RATIO;
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    driveTalon.getConfigurator().apply(config);
  }
}
