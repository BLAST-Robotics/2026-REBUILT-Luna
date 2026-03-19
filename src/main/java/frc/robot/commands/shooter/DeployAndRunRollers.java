package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class DeployAndRunRollers extends ParallelCommandGroup {
    public DeployAndRunRollers(IntakeSubsystem intake) {
        addCommands(
            new IntakePivotToState(intake, false),
            new RunIntakeRollers(intake, () -> IntakeConstants.ROLLER_DEFAULT_RPM)
        );
        // addRequirements(intake); // Removed to allow concurrent commands
    }
}
