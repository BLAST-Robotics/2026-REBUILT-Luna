package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntakeRollersVoltage extends Command {
    private final IntakeSubsystem intake;
    private final DoubleSupplier voltage;

    public RunIntakeRollersVoltage(IntakeSubsystem intake, DoubleSupplier voltagDoubleSupplier) {
        this.intake = intake;
        this.voltage = voltagDoubleSupplier;
        // addRequirements(intake); // Removed to allow concurrent commands
    }

    @Override
    public void execute() {
        // Use closed-loop velocity control instead of voltage to maintain speed under load
        intake.setRollerVoltage(voltage.getAsDouble());
        intake.setAgitatorVoltage(frc.robot.Constants.IntakeConstants.AGITATOR_VOLTS);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopRollers();
        intake.stopAgitator();
    }
}
