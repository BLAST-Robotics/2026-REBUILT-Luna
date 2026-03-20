package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class AutoShoot extends Command {
    private final ShooterSubsystem shooter;
    private final double targetRps;
    private final double durationSeconds;
    private final Timer timer = new Timer();

    public AutoShoot(ShooterSubsystem shooter, double rps, double seconds) {
        this.shooter = shooter;
        this.targetRps = rps;
        this.durationSeconds = seconds;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        shooter.setSpeed(targetRps);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(durationSeconds);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}
