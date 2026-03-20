package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.SwerveMath;

public final class Constants {

  public static final double ROBOT_MASS = (148 - 20.37) * 0.453592; // 32lbs * kg per lb
  public static final double CHASSIS_MASS = ROBOT_MASS;
  public static final Translation3d CHASSIS_CG = new Translation3d(0, 0, Units.inchesToMeters(8));
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED = Units.feetToMeters(14.5);

  public static final class OperatorConstants {
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
    public static final double LEFT_X_DEADBAND = 0.03;
    public static final double LEFT_Y_DEADBAND = 0.03;
    public static final double RIGHT_X_DEADBAND = 0.03;
    public static final double TURN_CONSTANT = 6;
  }

  public static final class DrivebaseConstants {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static final class ShooterConstants {
    public static final int RIGHT_MOTOR_ID = 9;
    public static final int LEFT_MOTOR_ID = 10;
    public static final int INDEXER_MOTOR_ID = 17;

    public static final double SHOOTER_kV = 0.12; // Starting value, tune as needed
    public static final double SHOOTER_kP = 0.11; // Starting value, tune as needed

    public static final double VELOCITY_THRESHOLD = 1.0; // RPS threshold for isUpToSpeed

    public static final double INDEXER_POWER = 0.8; // Open loop power for indexer
  }

  public static final class IntakeConstants {
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

  public static final class FieldConstants {

    public enum Zone {
      AZ, NZ, EZ
    }

    public static final Pose2d hubPositionBlue = new Pose2d(new Translation2d(4.625594, 4.0346376), Rotation2d.kZero);
    public static final Pose2d hubPositionRed = new Pose2d(new Translation2d(11.915394, 4.0346376), Rotation2d.kPi);
    public static final Pose2d blueShootPos = new Pose2d(
        new Translation2d(hubPositionBlue.getX() - 1.943 - 0.854, hubPositionBlue.getY() - 1.602 - 1.520 + 0.762),
        Rotation2d.kZero);

    public static final Pose2d redShootPos = new Pose2d(
        new Translation2d(hubPositionRed.getX() + 1.943 + 0.854, hubPositionRed.getY() + 1.602 + 1.520 - 0.762),
        Rotation2d.kPi);

    public static final double fieldWidth = 16.540988;
    public static final double fieldHeight = 8.069326;
    public static final Pose2d neutral = new Pose2d(new Translation2d(fieldWidth / 2.0, fieldHeight - 1.10),
        Rotation2d.kZero);

    public static final Pose2d[] blueBumpCenters = { new Pose2d(4.625594, 2.516886, new Rotation2d()),
        new Pose2d(4.625594, 5.5523892, new Rotation2d()) };
    public static final Pose2d[] redBumpCenters = { new Pose2d(11.915394, 2.516886, new Rotation2d()),
        new Pose2d(11.915394, 5.5523892, new Rotation2d()) };
    public static final Pose2d[] blueTrenchCenters = { new Pose2d(4.625594, 0.642493, new Rotation2d()),
        new Pose2d(4.625594, 7.4267822, new Rotation2d()) };
    public static final Pose2d[] redTrenchCenters = { new Pose2d(11.915394, 0.642493, new Rotation2d()),
        new Pose2d(11.915394, 7.4267822, new Rotation2d()) };
    public static final Pose2d[] allTrenchCenters = { FieldConstants.blueTrenchCenters[0],
        FieldConstants.blueTrenchCenters[1], FieldConstants.redTrenchCenters[0],
        FieldConstants.redTrenchCenters[1] };

    public static final double bumpLength = 1.82;
    public static final double bumpWidth = 1.1938;

    public static final double trenchLength = 1.281938;

    public static final Translation2d blueDepotCenter = new Translation2d(0, 5.9632596);
    public static final Translation2d redDepotCenter = new Translation2d(16.540988, 2.1060156);

    public static final Translation2d blueOutpostCenter = new Translation2d(0, 0.665988);
    public static final Translation2d redOutpostCenter = new Translation2d(16.540988, 7.4032872);

    public static final Translation2d blueTowerBarCenter = new Translation2d(1.139444, 3.7455856);
    public static final Translation2d redTowerBarCenter = new Translation2d(15.401544, 4.3236896);

    public static final double gravityEarth = 9.80665;
  }

}