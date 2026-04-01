package frc.robot;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.shooter.IntakePivotToState;
import frc.robot.commands.shooter.RunIntakeRollersVoltage;
import frc.robot.commands.shooter.RunShooterRPM;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;

public class RobotContainer
{
  private final SwerveSubsystem drivebase = OperatorConstants.SWERVE_ENABLED ? new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve")) : null;
  private final ShooterSubsystem shooter = OperatorConstants.SHOOTER_ENABLED ? new ShooterSubsystem() : null;
  private final IntakeSubsystem intake = OperatorConstants.INTAKE_ENABLED ? new IntakeSubsystem() : null;
  private final LEDSubsystem ledSubsystem = new LEDSubsystem(OperatorConstants.CANDLE_ID);

  private final SlewRateLimiter translationXLimiter = new SlewRateLimiter(9.0);
  private final SlewRateLimiter translationYLimiter = new SlewRateLimiter(9.0);
  private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(100.0);

  private boolean fieldCentric = true; // Robot-Centric "Arcade" default
  private boolean slewEnabled = true;
  private double flip = 1.0;
  private boolean invertTurn = false;

  private final SendableChooser<String> autonomousChooser = new SendableChooser<>();

  final CommandXboxController driverXbox = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
  final CommandXboxController operatorXbox = new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);
  private final SendableChooser<Command> autoChooser;

  public RobotContainer()
  {
    configureAutonomousChooser();
    configureBindings();
    registerPathPlannerNamedCommands();
    
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
              double speedMultiplier = (driverXbox.getHID().getRightBumper() || operatorXbox.getHID().getRightBumper()) ? 0.3 : 1.0;

              if (driverXbox.b().getAsBoolean()) { 
                  double forwardLimelight = LimelightHelpers.getTY("limelight") * kP_Range * (maxSpeed * speedMultiplier) * -1.0;
                  return slewEnabled ? -translationXLimiter.calculate(forwardLimelight) : -forwardLimelight;
              }
              // Normal forward on joystick gives negative Y. YAGSL needs positive for forward.
              double input = -MathUtil.applyDeadband(driverXbox.getLeftY() * flip, OperatorConstants.LEFT_Y_DEADBAND) * (maxSpeed * speedMultiplier);
              return slewEnabled ? translationXLimiter.calculate(input) : input;
          },
          // Translation Y (Left/Right Strafe) is Left X
          () -> {
              double speedMultiplier = (driverXbox.getHID().getRightBumper() || operatorXbox.getHID().getRightBumper()) ? 0.3 : 1.0;

              if (driverXbox.b().getAsBoolean()) {
                  return 0.0; // Stop strafing while auto-aiming
              }
                // Normal left on joystick gives negative X. YAGSL needs positive for left.
                // Note: was mathUtil minus, changing back to respect flip cleanly
                double input = -MathUtil.applyDeadband(driverXbox.getLeftX() * flip, OperatorConstants.LEFT_X_DEADBAND) * (maxSpeed * speedMultiplier);
                return slewEnabled ? translationYLimiter.calculate(input) : input;

      
          },
          // Angular Rotation (Turn)
          () -> {
              double speedMultiplier = (driverXbox.getHID().getRightBumper() || operatorXbox.getHID().getRightBumper()) ? 0.3 : 1.0;
              double turnMultiplier = invertTurn ? -1.0 : 1.0;

              if (driverXbox.b().getAsBoolean()) { 
                  double rotLimelight = LimelightHelpers.getTX("limelight") * kP_Aim * (maxAngularVelocity * speedMultiplier) * -1.0;
                  return slewEnabled ? rotationLimiter.calculate(rotLimelight) : rotLimelight;
              }
              // Invert Turn to make right negative rotation
              double input = -MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND) * (maxAngularVelocity * speedMultiplier) * turnMultiplier;
              return slewEnabled ? rotationLimiter.calculate(input) : input;
          },
          () -> fieldCentric
        );

        drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
    
        SmartDashboard.putBoolean("Field Centric", fieldCentric);
        SmartDashboard.putNumber("Swerve Flipped", flip);
        SmartDashboard.putBoolean("Invert Turn", invertTurn);
        
        drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
    } else {
        autoChooser = new SendableChooser<>();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    SmartDashboard.putData("Autonomous Mode", autonomousChooser);
  }

  private void configureAutonomousChooser()
  {
    autonomousChooser.setDefaultOption("Hub Side Auto Safe", "Hub Side Auto Safe");
    autonomousChooser.addOption("Depot Side Auto Safe", "Depot Side Auto Safe");
    autonomousChooser.addOption("Outpost Side Auto Safe", "Outpost Side Auto Safe");
  }

  private void registerPathPlannerNamedCommands()
  {
    if (shooter != null) {
      NamedCommands.registerCommand("shoot", new RunShooterRPM(shooter, intake, () -> frc.robot.Constants.ShooterConstants.SHOOTER_DEFAULT_RPM/60));
      NamedCommands.registerCommand("endShoot", Commands.runOnce(() -> shooter.stop(), shooter));
    }
  }
  
  private void configureBindings()
  {
    // Driver & Operator Config: Both can handle the same functions
    // Removed Invert Drive toggle from D-Pad Left.
    
    // D-Pad Left: Pivot intake up and down to compress storage
    // Removed compress storage as requested.

    // D-Pad Right: Toggle Field/Robot Centric (Drives like Arcade)
    Command toggleFieldCentric = Commands.runOnce(() -> {
      fieldCentric = !fieldCentric;
      SmartDashboard.putBoolean("Field Centric", fieldCentric);
    });
    //driverXbox.povRight().onTrue(toggleFieldCentric);
    //operatorXbox.povRight().onTrue(toggleFieldCentric);
    
    Command invert = Commands.runOnce(()->{
      flip*=-1;
      SmartDashboard.putNumber("Swerve Flipped", flip);
    });
    driverXbox.leftBumper().onTrue(invert);
    operatorXbox.leftBumper().onTrue(invert);

    // D-Pad Left: Toggle Invert Turn
    Command toggleInvertTurn = Commands.runOnce(() -> {
      invertTurn = !invertTurn;
      SmartDashboard.putBoolean("Invert Turn", invertTurn);
    });
    driverXbox.leftTrigger().onTrue(toggleInvertTurn);
    operatorXbox.leftTrigger().onTrue(toggleInvertTurn);

    // Back Button (View): Toggle Slew Rate Limiter
    Command toggleSlew = Commands.runOnce(() -> {
      slewEnabled = !slewEnabled;
      SmartDashboard.putBoolean("Slew Enabled", slewEnabled);
    });
    driverXbox.back().onTrue(toggleSlew);
    operatorXbox.back().onTrue(toggleSlew);

    if (shooter != null) {
      driverXbox.y().toggleOnTrue(new RunShooterRPM(shooter, intake, () -> 47));
      operatorXbox.y().toggleOnTrue(new RunShooterRPM(shooter, intake, () -> 47));
    }
    
    if (intake != null) {
      Command runIntake = new RunIntakeRollersVoltage(intake, () -> 5);
      driverXbox.x().toggleOnTrue(runIntake);
      operatorXbox.x().toggleOnTrue(runIntake);

      Command runAgitator = Commands.startEnd(
        () -> intake.setAgitatorVoltage(frc.robot.Constants.IntakeConstants.AGITATOR_VOLTS), 
        () -> intake.stopAgitator()
      );
      // driverXbox.leftTrigger().whileTrue(runAgitator);
      // operatorXbox.leftTrigger().whileTrue(runAgitator);

      SmartDashboard.putData("Toggle Agitators Enable", Commands.runOnce(intake::toggleAgitators));

      // Intake Reverse on Right Trigger
      Command reverseIntake = Commands.startEnd(
        () -> {
          intake.setRollerVoltage(frc.robot.Constants.IntakeConstants.ROLLER_REVERSE_VOLTAGE);  
          intake.setAgitatorVoltage(frc.robot.Constants.IntakeConstants.AGITATOR_REVERSE_VOLTS);
        },
        () -> {
          intake.stopRollers();
          intake.stopAgitator();
        }
      );
      driverXbox.a().whileTrue(reverseIntake);
      operatorXbox.a().whileTrue(reverseIntake);
      
      // Reversed intake pivot bindings
      Command pivotUp = new IntakePivotToState(intake, true, false);
      Command pivotDown = new IntakePivotToState(intake, false, false);
      
      driverXbox.povUp().whileTrue(pivotUp);
      driverXbox.povDown().whileTrue(pivotDown);      
      operatorXbox.povUp().whileTrue(pivotUp);        
      operatorXbox.povDown().whileTrue(pivotDown);
    }

    if (drivebase != null) {
      Command zeroGyro = Commands.runOnce(drivebase::zeroGyroWithAlliance, drivebase);
      driverXbox.rightTrigger().onTrue(zeroGyro);
      operatorXbox.rightTrigger().onTrue(zeroGyro);
      
      // Removed the redundant x-instance lock command as requested
    }
  }

  public Command zero() {
    if (drivebase != null) {
      drivebase.zeroGyro();
    }
    return Commands.none();
  }

  public Command getAutonomousCommand()
  {
    String selectedAuto = autonomousChooser.getSelected();

    if (selectedAuto != null) {
      Command command = AutoBuilder.buildAuto(selectedAuto);
      if (command != null) {
        try {
          PathPlannerAuto pathPlannerAuto = new PathPlannerAuto(selectedAuto);
          Pose2d startingPose = pathPlannerAuto.getStartingPose();
          if (startingPose != null && drivebase != null) {
            return Commands.runOnce(() -> drivebase.resetOdometry(startingPose), drivebase).andThen(command);
          }
        } catch (Exception e) {}
        return command;
      }
    }

    return autoChooser.getSelected();
  }

  public void setDriveMode()
  {
    // Used to reset drive modes in teleop init
  }

  public void setMotorBrake(boolean brake)
  {
    if (drivebase != null) {
      drivebase.setMotorBrake(brake);
    }
  }
}