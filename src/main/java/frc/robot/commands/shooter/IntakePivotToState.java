package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakePivotToState extends Command {
    private final IntakeSubsystem intake;
    private final boolean up;
    private final boolean brake;
    private final Timer timer = new Timer();

    public IntakePivotToState(IntakeSubsystem intake, boolean up, boolean brake) {
        this.intake = intake;
        this.up = up;
        this.brake = brake;
        // addRequirements(intake); // Removed to allow concurrent commands
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        double targetPower = up ? IntakeConstants.PIVOT_UP_POWER : IntakeConstants.PIVOT_DOWN_POWER;
        // Ramp up from 0 to 1.0 over PIVOT_RAMP_TIME seconds
        double rampFactor = Math.min(timer.get() / IntakeConstants.PIVOT_RAMP_TIME, 1.0);
        intake.setPivotPower(targetPower * rampFactor);
    }

    @Override
    public boolean isFinished() {
        // Run while the button is held (configured in RobotContainer)
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setPivotPower(0);
        intake.setPivotNeutralMode(brake);
    }
}
