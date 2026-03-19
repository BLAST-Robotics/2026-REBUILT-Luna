package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RunShooterRPM extends Command {
    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private final DoubleSupplier rpsSupplier;

    public RunShooterRPM(ShooterSubsystem shooter, IntakeSubsystem intake, DoubleSupplier rpsSupplier) {
        this.shooter = shooter;
        this.intake = intake;
        this.rpsSupplier = rpsSupplier;
        if (intake != null) {
            // addRequirements(shooter, intake); // Removed intake to allow concurrent commands
            addRequirements(shooter);
        } else {
            addRequirements(shooter);
        }
    }

    @Override
    public void execute() {
        shooter.setSpeed(rpsSupplier.getAsDouble());
        if (intake != null) {
            intake.setAgitatorVoltage(frc.robot.Constants.IntakeConstants.AGITATOR_VOLTS);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        if (intake != null) {
            intake.stopAgitator();
        }
    }
}
