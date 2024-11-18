package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

public final class IntakeConstants {

  // Constantes para os sensores
  public static final int INSIDE_SENSOR_CHANNEL = 0;
  public static final int OUTSIDE_SENSOR_CHANNEL = 1;

  // Constantes para os solenoides
  public static final int SOLENOID_MODULE_CAN_ID = 19;
  public static final PneumaticsModuleType SOLENOID_MODULE_TYPE = PneumaticsModuleType.REVPH;
  public static final int SOLENOID_LEFT_FORWARD_CHANNEL = 0;
  public static final int SOLENOID_LEFT_REVERSE_CHANNEL = 1;
  public static final int SOLENOID_RIGHT_FORWARD_CHANNEL = 2;
  public static final int SOLENOID_RIGHT_REVERSE_CHANNEL = 3;
}
