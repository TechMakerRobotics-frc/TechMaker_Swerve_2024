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
import frc.robot.subsystems.drive.DriveConstants.ModuleConstants;

public class ModuleIOSparkAndTalon implements ModuleIO {

  private static final double DRIVE_GEAR_RATIO = ModuleConstants.DRIVE_GEAR_RATIO;
  private static final double TURN_GEAR_RATIO = ModuleConstants.TURN_GEAR_RATIO;

  private final CANSparkMax turnSparkMax;
  private final RelativeEncoder turnRelativeEncoder;
  private final CANcoder cancoder;
  private final StatusSignal<Double> turnAbsolutePosition;
  private final TalonFX driveTalon;

  private boolean isTurnMotorInverted = ModuleConstants.TURN_MOTOR_INVERTED;
  private final Rotation2d absoluteEncoderOffset;

  private final StatusSignal<Double> drivePosition;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveCurrent;

  public ModuleIOSparkAndTalon(int index) {
    switch (index) {
      case 0: // front left
        driveTalon = new TalonFX(ModuleConstants.TALON_FL, ModuleConstants.CANBUS);
        turnSparkMax = new CANSparkMax(ModuleConstants.SPARK_FL, MotorType.kBrushless);
        cancoder = new CANcoder(ModuleConstants.CANCODER_FL, ModuleConstants.CANBUS);
        absoluteEncoderOffset = new Rotation2d(ModuleConstants.ENCODER_OFFSET_FL);
        break;
      case 1: // front right
        driveTalon = new TalonFX(ModuleConstants.TALON_FR, ModuleConstants.CANBUS);
        turnSparkMax = new CANSparkMax(ModuleConstants.SPARK_FR, MotorType.kBrushless);
        cancoder = new CANcoder(ModuleConstants.CANCODER_FR, ModuleConstants.CANBUS);
        absoluteEncoderOffset = new Rotation2d(ModuleConstants.ENCODER_OFFSET_FR);
        break;
      case 2: // back left
        driveTalon = new TalonFX(ModuleConstants.TALON_BL, ModuleConstants.CANBUS);
        turnSparkMax = new CANSparkMax(ModuleConstants.SPARK_BL, MotorType.kBrushless);
        cancoder = new CANcoder(ModuleConstants.CANCODER_BL, ModuleConstants.CANBUS);
        absoluteEncoderOffset = new Rotation2d(ModuleConstants.ENCODER_OFFSET_BL);
        break;
      case 3: // back right
        driveTalon = new TalonFX(ModuleConstants.TALON_BR, ModuleConstants.CANBUS);
        turnSparkMax = new CANSparkMax(ModuleConstants.SPARK_BR, MotorType.kBrushless);
        cancoder = new CANcoder(ModuleConstants.CANCODER_BR, ModuleConstants.CANBUS);
        absoluteEncoderOffset = new Rotation2d(ModuleConstants.ENCODER_OFFSET_BR);
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    // Initialize TalonFX configuration
    var driveConfig = new TalonFXConfiguration();
    driveConfig.CurrentLimits.SupplyCurrentLimit = ModuleConstants.SUPPLY_CURRENT_LIMIT;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable =
        ModuleConstants.SUPPLY_CURRENT_LIMIT_ENABLE;
    driveTalon.getConfigurator().apply(driveConfig);
    setDriveBrakeMode(false);

    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        ModuleConstants.ODOMETRY_UPDATE_FREQUENCY, drivePosition); // Required for odometry
    BaseStatusSignal.setUpdateFrequencyForAll(
        ModuleConstants.OTHER_SIGNALS_UPDATE_FREQUENCY,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent);

    turnSparkMax.restoreFactoryDefaults();
    turnSparkMax.setCANTimeout(ModuleConstants.CAN_TIMEOUT_MS);
    turnRelativeEncoder = turnSparkMax.getEncoder();
    turnSparkMax.setInverted(isTurnMotorInverted);
    turnSparkMax.setSmartCurrentLimit(ModuleConstants.SMART_CURRENT_LIMIT);
    turnSparkMax.enableVoltageCompensation(ModuleConstants.VOLTAGE_COMPENSATION);
    turnRelativeEncoder.setPosition(ModuleConstants.INITIAL_ENCODER_POSITION);
    turnRelativeEncoder.setMeasurementPeriod(ModuleConstants.ENCODER_MEASUREMENT_PERIOD_MS);
    turnRelativeEncoder.setAverageDepth(ModuleConstants.ENCODER_AVERAGE_DEPTH);
    turnSparkMax.burnFlash();

    cancoder.getConfigurator().apply(new CANcoderConfiguration());
    turnAbsolutePosition = cancoder.getAbsolutePosition();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Update drive (TalonFX)
    BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent);
    inputs.drivePositionRad =
        Units.rotationsToRadians(drivePosition.getValueAsDouble()) / DRIVE_GEAR_RATIO;
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(driveVelocity.getValueAsDouble())
            / DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = new double[] {driveCurrent.getValueAsDouble()};

    // Update turn (CANSparkMax)
    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
            .minus(absoluteEncoderOffset);
    inputs.turnPosition =
        Rotation2d.fromRotations(turnRelativeEncoder.getPosition() / TURN_GEAR_RATIO);
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
            / TURN_GEAR_RATIO;
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
