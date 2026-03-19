package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class RunShooterVoltage extends Command {
    private final ShooterSubsystem shooter;
    private final DoubleSupplier voltageSupplier;

    public RunShooterVoltage(ShooterSubsystem shooter, DoubleSupplier voltageSupplier) {
        this.shooter = shooter;
        this.voltageSupplier = voltageSupplier;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setVoltage(voltageSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}
