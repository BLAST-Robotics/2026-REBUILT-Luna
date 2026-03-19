package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.SwerveMath;

public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.37) * 0.453592; // 32lbs * kg per lb
  public static final double CHASSIS_MASS = ROBOT_MASS;
  public static final Translation3d CHASSIS_CG = new Translation3d(0, 0, Units.inchesToMeters(8));
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);

  public static final class OperatorConstants
  {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    // Set this to true to use only one controller, false to use two.
    public static final boolean SINGLE_CONTROLLER_MODE = true;

    // Set this to true to completely disable auto-aim features.
    public static final boolean AUTO_AIM_DISABLED = true;

    // Subsystem Initialization Toggles
    public static final boolean SWERVE_ENABLED = true;
    public static final boolean INTAKE_ENABLED = true;
    public static final boolean SHOOTER_ENABLED = true;
    public static final boolean VISION_ENABLED = true;
    public static final boolean AGITATOR_ENABLED = false;

    // CAN IDs
    public static final int CANDLE_ID = 30;

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.03;
    public static final double LEFT_Y_DEADBAND  = 0.03;
    public static final double RIGHT_X_DEADBAND = 0.03;
    public static final double TURN_CONSTANT    = 6;
  }

  public static final class DrivebaseConstants
  {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static final class ShooterConstants
  {
    public static final int RIGHT_MOTOR_ID = 9;
    public static final int LEFT_MOTOR_ID = 10;
    public static final int INDEXER_MOTOR_ID = 17;

    public static final double SHOOTER_kV = 0.12; // Starting value, tune as needed
    public static final double SHOOTER_kP = 0.11; // Starting value, tune as needed

    public static final double VELOCITY_THRESHOLD = 1.0; // RPS threshold for isUpToSpeed

    public static final double INDEXER_POWER = 0.8; // Open loop power for indexer
  }

  public static final class IntakeConstants
  {
    public static final int PIVOT_ID = 18;
    public static final int ROLLER_ID = 16;
    public static final int AGITATOR_ID = 15;

    public static final double PIVOT_GEAR_RATIO = 52.5;
    public static final double ROLLER_GEAR_RATIO = 2.0;

    public static final double PIVOT_UP_POWER = 0.2;
    public static final double PIVOT_DOWN_POWER = -0.2; // Reversed as requested
    public static final double PIVOT_RAMP_TIME = 0.5; // Seconds to ramp up

    public static final double ROLLER_DEFAULT_RPM = 2500.0;
    
    public static final double AGITATOR_VOLTS = 11.0;
    public static final boolean AGITATOR_INVERT = true;
  }

  public static final class FieldConstants
  {
    public static final double FIELD_LENGTH = 16.541;
    public static final double HUB_X_OFFSET = 4.63;
    public static final double HUB_Y = 4.1;
    public static final double HUB_SIDE_Y_UP = 5.5;
    public static final double HUB_SIDE_Y_DOWN = 2.5;
    public static final double TARGET_DISTANCE_TO_HUB = Units.feetToMeters(8.0);
  }
}