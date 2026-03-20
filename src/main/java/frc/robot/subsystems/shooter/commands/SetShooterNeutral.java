package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * A command to set the shooter motors to either Brake or Coast mode.
 */
public class SetShooterNeutral extends InstantCommand {
    /**
     * @param shooter The ShooterSubsystem
     * @param brake True for Brake mode, false for Coast mode.
     */
    public SetShooterNeutral(ShooterSubsystem shooter, boolean brake) {
        super(() -> shooter.setNeutralMode(brake), shooter);
    }
}
