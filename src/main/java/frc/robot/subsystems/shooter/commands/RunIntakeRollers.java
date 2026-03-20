package frc.robot.subsystems.shooter.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntakeRollers extends Command {
    private final IntakeSubsystem intake;
    private final DoubleSupplier rpmSupplier;

    public RunIntakeRollers(IntakeSubsystem intake, DoubleSupplier rpmSupplier) {
        this.intake = intake;
        this.rpmSupplier = rpmSupplier;
        // addRequirements(intake); // Removed to allow concurrent commands
    }

    @Override
    public void execute() {
        // Use closed-loop velocity control instead of voltage to maintain speed under load
        intake.setRollerRPM(rpmSupplier.getAsDouble());
        intake.setAgitatorVoltage(frc.robot.Constants.IntakeConstants.AGITATOR_VOLTS);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopRollers();
        intake.stopAgitator();
    }
}
