package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class SetIntakeNeutralMode extends InstantCommand {
    public SetIntakeNeutralMode(IntakeSubsystem intake, boolean brake) {
        super(() -> intake.setPivotNeutralMode(brake), intake);
    }
}
