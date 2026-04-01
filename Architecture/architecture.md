# FRC Robot Code Architecture

This document contains the class diagram for the FRC robot code in the `frc.robot` package.

```mermaid
classDiagram
    class TimedRobot {
        +robotInit()
        +robotPeriodic()
        +autonomousInit()
        +autonomousPeriodic()
        +teleopInit()
        +teleopPeriodic()
        +disabledInit()
        +disabledPeriodic()
        +testInit()
        +testPeriodic()
        +simulationInit()
        +simulationPeriodic()
    }

    class SubsystemBase {
        +periodic()
        +setDefaultCommand(Command)
        +getDefaultCommand()
    }

    class Command {
        +initialize()
        +execute()
        +isFinished()
        +end(boolean)
        +runsWhenDisabled()
    }

    class InstantCommand {
        +initialize()
        +execute()
        +isFinished()
        +end(boolean)
    }

    class ParallelCommandGroup {
        +initialize()
        +execute()
        +isFinished()
        +end(boolean)
    }

    class Robot {
        -RobotContainer m_robotContainer
        -Command m_autonomousCommand
        -Timer disabledTimer
        +robotInit()
        +robotPeriodic()
        +autonomousInit()
        +autonomousPeriodic()
        +teleopInit()
        +teleopPeriodic()
        +disabledInit()
        +disabledPeriodic()
        +testInit()
        +testPeriodic()
        +simulationInit()
        +simulationPeriodic()
    }

    class RobotContainer {
        -SwerveSubsystem drivebase
        -ShooterSubsystem shooter
        -IntakeSubsystem intake
        -LEDSubsystem ledSubsystem
        -SendableChooser autonomousChooser
        -SendableChooser autoChooser
        +configureBindings()
        +getAutonomousCommand()
        +setMotorBrake(boolean)
        +setDriveMode()
    }

    class SwerveSubsystem {
        -SwerveDrive swerveDrive
        -SwerveController controller
        +driveCommand()
        +getPose()
        +resetOdometry()
        +zeroGyro()
    }

    class ShooterSubsystem {
        +runShooterRPM(double)
        +runShooterVoltage(double)
        +stopShooter()
        +getShooterRPM()
    }

    class IntakeSubsystem {
        +runIntakeRollers(double)
        +runIntakeRollersVoltage(double)
        +stopIntakeRollers()
        +setIntakePivot(double)
        +getIntakePivotPosition()
    }

    class LEDSubsystem {
        +setLEDColor(int, int, int)
        +setLEDPattern(String)
    }

    class VisionSubsystem {
        +getTargetData()
        +isTargetVisible()
    }

    class ExampleSubsystem {
        +exampleMethod()
        +exampleMethodCommand()
    }

    class ExampleCommand {
        +initialize()
        +execute()
        +isFinished()
        +end(boolean)
    }

    class AutoShoot {
        +initialize()
        +execute()
        +isFinished()
        +end(boolean)
    }

    class DeployAndRunRollers {
        +initialize()
        +execute()
        +isFinished()
        +end(boolean)
    }

    class IntakePivotToState {
        +initialize()
        +execute()
        +isFinished()
        +end(boolean)
    }

    class RunIntakeRollers {
        +initialize()
        +execute()
        +isFinished()
        +end(boolean)
    }

    class RunIntakeRollersVoltage {
        +initialize()
        +execute()
        +isFinished()
        +end(boolean)
    }

    class RunShooterRPM {
        +initialize()
        +execute()
        +isFinished()
        +end(boolean)
    }

    class RunShooterVoltage {
        +initialize()
        +execute()
        +isFinished()
        +end(boolean)
    }

    class SetIntakeNeutralMode {
        +initialize()
        +execute()
        +isFinished()
        +end(boolean)
    }

    class SetShooterNeutral {
        +initialize()
        +execute()
        +isFinished()
        +end(boolean)
    }

    class Autos {
        +exampleAuto(ExampleSubsystem)
    }

    class LimelightHelpers {
        +getTX(String)
        +getTY(String)
        +getTA(String)
        +getTS(String)
    }

    class Constants {
        +MAX_SPEED
        +DrivebaseConstants
        +OperatorConstants
    }

    class Main {
        +main(String[])
    }

    class Elastic {
        +sendTab(String, Object)
        +sendTabGroup(String, Object[])
    }

    TimedRobot <|-- Robot : extends
    SubsystemBase <|-- SwerveSubsystem : extends
    SubsystemBase <|-- ShooterSubsystem : extends
    SubsystemBase <|-- IntakeSubsystem : extends
    SubsystemBase <|-- LEDSubsystem : extends
    SubsystemBase <|-- VisionSubsystem : extends
    SubsystemBase <|-- ExampleSubsystem : extends

    Command <|-- ExampleCommand : extends
    Command <|-- AutoShoot : extends
    ParallelCommandGroup <|-- DeployAndRunRollers : extends
    Command <|-- IntakePivotToState : extends
    Command <|-- RunIntakeRollers : extends
    Command <|-- RunIntakeRollersVoltage : extends
    Command <|-- RunShooterRPM : extends
    Command <|-- RunShooterVoltage : extends
    InstantCommand <|-- SetIntakeNeutralMode : extends
    InstantCommand <|-- SetShooterNeutral : extends

    Robot --> RobotContainer : has
    RobotContainer --> SwerveSubsystem : has
    RobotContainer --> ShooterSubsystem : has
    RobotContainer --> IntakeSubsystem : has
    RobotContainer --> LEDSubsystem : has

    ExampleCommand --> ExampleSubsystem : uses
    AutoShoot --> ShooterSubsystem : uses
    DeployAndRunRollers --> IntakeSubsystem : uses
    IntakePivotToState --> IntakeSubsystem : uses
    RunIntakeRollers --> IntakeSubsystem : uses
    RunIntakeRollersVoltage --> IntakeSubsystem : uses
    RunShooterRPM --> ShooterSubsystem : uses
    RunShooterVoltage --> ShooterSubsystem : uses
    SetIntakeNeutralMode --> IntakeSubsystem : uses
    SetShooterNeutral --> ShooterSubsystem : uses

    Autos --> ExampleSubsystem : uses
    Autos --> ExampleCommand : uses

    SwerveSubsystem --> VisionSubsystem : uses
    RobotContainer --> VisionSubsystem : uses
```

## Robot.java Execution Flow

This flowchart shows the execution flow of the Robot.java class, focusing on the different modes (Autonomous, Manual/Teleop, Disabled, Test, Simulation).

```mermaid
flowchart TD
    A["Robot Power On"] --> B["robotInit()"]
    B --> C["RobotContainer Instantiated"]
    C --> D["robotPeriodic() - Every 20ms"]
    
    D --> E{"Driver Station Mode"}
    
    E -->|"Disabled"| F["disabledInit()"]
    F --> G["disabledPeriodic()"]
    G --> D
    
    E -->|"Autonomous"| H["autonomousInit()"]
    H --> I["Get Autonomous Command from RobotContainer"]
    I --> J["Schedule Autonomous Command"]
    J --> K["autonomousPeriodic()"]
    K --> D
    
    E -->|"Teleop"| L["teleopInit()"]
    L --> M["Cancel Autonomous Command"]
    M --> N["Set Drive Mode"]
    N --> O["teleopPeriodic()"]
    O --> D
    
    E -->|"Test"| P["testInit()"]
    P --> Q["testPeriodic()"]
    Q --> D
    
    E -->|"Simulation"| R["simulationInit()"]
    R --> S["simulationPeriodic()"]
    S --> D
```

### Flow Explanation

**Initialization:**
- `robotInit()`: Called once at startup, creates RobotContainer and initializes systems

**Main Loop:**
- `robotPeriodic()`: Runs every 20ms, executes Command Scheduler for all modes

**Autonomous Mode:**
- `autonomousInit()`: Gets and schedules autonomous command from RobotContainer
- `autonomousPeriodic()`: Command Scheduler handles autonomous execution

**Manual (Teleop) Mode:**
- `teleopInit()`: Cancels autonomous commands, sets drive mode for manual control
- `teleopPeriodic()`: Command Scheduler handles driver input and commands

**Other Modes:**
- Disabled: Motor brake control with timer
- Test: Command cancellation and swerve parser initialization
- Simulation: Simulation-specific initialization and periodic functions

## Overview

This diagram represents the WPILib command-based robot architecture for the FRC robot code:

- **Robot**: Main robot class extending `TimedRobot`
- **RobotContainer**: Container class managing subsystems and commands
- **Subsystems**: Hardware control classes extending `SubsystemBase`
- **Commands**: Action classes extending `Command` or its variants
- **Utilities**: Helper classes like `LimelightHelpers`, `Constants`, etc.

The diagram shows inheritance relationships (using `<|--`) and composition/usage relationships (using `-->`) between the classes.