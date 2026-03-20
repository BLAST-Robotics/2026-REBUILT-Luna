package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.commands.IntakePivotToState;
import frc.robot.subsystems.shooter.commands.RunIntakeRollers;
import frc.robot.subsystems.shooter.commands.RunShooterRPM;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

public class RobotContainer {
  private final SwerveSubsystem drivebase = OperatorConstants.SWERVE_ENABLED
      ? new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"))
      : null;
  private final ShooterSubsystem shooter = OperatorConstants.SHOOTER_ENABLED ? new ShooterSubsystem() : null;
  private final IntakeSubsystem intake = OperatorConstants.INTAKE_ENABLED ? new IntakeSubsystem() : null;
  private final LEDSubsystem ledSubsystem = new LEDSubsystem(OperatorConstants.CANDLE_ID);

  private final SlewRateLimiter translationXLimiter = new SlewRateLimiter(2.0);
  private final SlewRateLimiter translationYLimiter = new SlewRateLimiter(2.0);
  private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(2.0);

  private boolean fieldCentric = true; // Robot-Centric "Arcade" default
  private boolean invertDrive = false;
  private boolean slewEnabled = true;

  final CommandXboxController driverXbox = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
  final CommandXboxController operatorXbox = new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    configureBindings();

    // Build PathPlanner Auto Chooser
    if (drivebase != null) {

      autoChooser = AutoBuilder.buildAutoChooser("Default Auto"); // Loads from deploy/pathplanner/autos
      SmartDashboard.putData("Auto Chooser", autoChooser);

      // Proportional Control Constants (Tune these!)
      final double kP_Aim = 0.035;
      final double kP_Range = 0.1;
      final double maxSpeed = Constants.MAX_SPEED;
      final double maxAngularVelocity = drivebase.getSwerveDrive().getMaximumChassisAngularVelocity();

      Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
          // Translation X (Forward/Backward) is Left Y
          () -> {
            // SLOW MODE MULTIPLIER
            double speedMultiplier = (driverXbox.rightBumper().getAsBoolean()
                || operatorXbox.rightBumper().getAsBoolean()) ? 0.3 : 1.0;

            if (driverXbox.a().getAsBoolean()) {
              double forwardLimelight = LimelightHelpers.getTY("limelight") * kP_Range * (maxSpeed * speedMultiplier)
                  * -1.0;
              return slewEnabled ? -translationXLimiter.calculate(forwardLimelight) : -forwardLimelight;
            }
            // Normal forward on joystick gives negative Y. YAGSL needs positive for
            // forward.
            double input = -MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND)
                * (maxSpeed * speedMultiplier);
            return slewEnabled ? translationXLimiter.calculate(input) : input;
          },
          // Translation Y (Left/Right Strafe) is Left X
          () -> {
            double speedMultiplier = (driverXbox.rightBumper().getAsBoolean()
                || operatorXbox.rightBumper().getAsBoolean()) ? 0.3 : 1.0;

            if (driverXbox.a().getAsBoolean()) {
              return 0.0; // Stop strafing while auto-aiming
            }
            // Normal left on joystick gives negative X. YAGSL needs positive for left.
            double input = -MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND)
                * (maxSpeed * speedMultiplier);
            return slewEnabled ? translationYLimiter.calculate(input) : input;
          },
          // Angular Rotation (Turn)
          () -> {
            double speedMultiplier = (driverXbox.rightBumper().getAsBoolean()
                || operatorXbox.rightBumper().getAsBoolean()) ? 0.3 : 1.0;

            if (driverXbox.a().getAsBoolean()) {
              double rotLimelight = LimelightHelpers.getTX("limelight") * kP_Aim
                  * (maxAngularVelocity * speedMultiplier) * -1.0;
              return slewEnabled ? rotationLimiter.calculate(rotLimelight) : rotLimelight;
            }
            // Invert Turn to make right negative rotation
            double input = -MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND)
                * (maxAngularVelocity * speedMultiplier);
            return slewEnabled ? rotationLimiter.calculate(input) : input;
          },
          () -> fieldCentric);

      drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);

      SmartDashboard.putBoolean("Field Centric", fieldCentric);
      SmartDashboard.putBoolean("Invert Drive", invertDrive);

      drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
    } else {
      autoChooser = new SendableChooser<>();
      SmartDashboard.putData("Auto Chooser", autoChooser);
    }
    shooter.setDefaultCommand(runTurretCommand());
  }

  private void configureBindings() {
    // Driver & Operator Config: Both can handle the same functions
    // Removed Invert Drive toggle from D-Pad Left.

    // D-Pad Left: Pivot intake up and down to compress storage
    // Removed compress storage as requested.

    // D-Pad Right: Toggle Field/Robot Centric (Drives like Arcade)
    Command toggleFieldCentric = Commands.runOnce(() -> {
      fieldCentric = !fieldCentric;
      SmartDashboard.putBoolean("Field Centric", fieldCentric);
    });
    driverXbox.povRight().onTrue(toggleFieldCentric);
    operatorXbox.povRight().onTrue(toggleFieldCentric);

    // D-Pad Left: Toggle Slew Rate Limiter
    Command toggleSlew = Commands.runOnce(() -> {
      slewEnabled = !slewEnabled;
      SmartDashboard.putBoolean("Slew Enabled", slewEnabled);
    });
    driverXbox.povLeft().onTrue(toggleSlew);
    operatorXbox.povLeft().onTrue(toggleSlew);

    if (shooter != null) {
      driverXbox.y().whileTrue(new RunShooterRPM(shooter, intake, () -> 47));
      operatorXbox.y().whileTrue(new RunShooterRPM(shooter, intake, () -> 47));
    }

    if (intake != null) {
      driverXbox.x()
          .toggleOnTrue(new RunIntakeRollers(intake, () -> frc.robot.Constants.IntakeConstants.ROLLER_DEFAULT_RPM));
      operatorXbox.x()
          .toggleOnTrue(new RunIntakeRollers(intake, () -> frc.robot.Constants.IntakeConstants.ROLLER_DEFAULT_RPM));

      Command runAgitator = Commands.startEnd(
          () -> intake.setAgitatorVoltage(frc.robot.Constants.IntakeConstants.AGITATOR_VOLTS),
          () -> intake.stopAgitator());
      driverXbox.leftTrigger().whileTrue(runAgitator);
      operatorXbox.leftTrigger().whileTrue(runAgitator);

      SmartDashboard.putData("Toggle Agitators Enable", Commands.runOnce(intake::toggleAgitators));

      // Intake Reverse on Right Trigger
      Command reverseIntake = Commands.startEnd(
          () -> {
            intake.setRollerRPM(-frc.robot.Constants.IntakeConstants.ROLLER_DEFAULT_RPM);
            intake.setAgitatorVoltage(-frc.robot.Constants.IntakeConstants.AGITATOR_VOLTS);
          },
          () -> {
            intake.stopRollers();
            intake.stopAgitator();
          });
      driverXbox.rightTrigger().whileTrue(reverseIntake);
      operatorXbox.rightTrigger().whileTrue(reverseIntake);

      // Reversed intake pivot bindings
      driverXbox.povUp().whileTrue(new IntakePivotToState(intake, true));
      driverXbox.povDown().whileTrue(new IntakePivotToState(intake, false));
      operatorXbox.povUp().whileTrue(new IntakePivotToState(intake, true));
      operatorXbox.povDown().whileTrue(new IntakePivotToState(intake, false));
    }

    if (drivebase != null) {
      Command zeroGyro = Commands.runOnce(drivebase::zeroGyroWithAlliance, drivebase);
      driverXbox.a().onTrue(zeroGyro);
      operatorXbox.a().onTrue(zeroGyro);

      // The auto-aim drive command
      // Removed the auto-aim logic as requested.

      Command lockDriveBase = Commands.runOnce(drivebase::lock, drivebase).repeatedly();
      driverXbox.x().whileTrue(lockDriveBase);
      operatorXbox.x().whileTrue(lockDriveBase);
    }
  }

  public Command zero() {
    if (drivebase != null) {
      drivebase.zeroGyro();
    }
    return Commands.none();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void setDriveMode() {
    // Used to reset drive modes in teleop init
  }

  public void setMotorBrake(boolean brake) {
    if (drivebase != null) {
      drivebase.setMotorBrake(brake);
    }
  }

  public RunCommand runTurretCommand() {
    return new RunCommand(
        () -> shooter.setSpeed(shooter.getShooterSpeed(drivebase.getPose())),
        shooter);
  }
}