package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    // Pivot: SparkMax + NEO
    private final SparkMax pivotMotor = new SparkMax(IntakeConstants.PIVOT_ID, MotorType.kBrushless);
    private final SparkMaxConfig pivotConfig = new SparkMaxConfig();

    // Rollers: SparkMax + NEO
    private final SparkMax rollerMotor = new SparkMax(IntakeConstants.ROLLER_ID, MotorType.kBrushless);
    private final SparkMaxConfig rollerConfig = new SparkMaxConfig();
    private final SparkClosedLoopController rollerController = rollerMotor.getClosedLoopController();

    // Agitator: SparkMax + NEO
    private final SparkMax agitatorMotor = new SparkMax(IntakeConstants.AGITATOR_ID, MotorType.kBrushless);
    private final SparkMaxConfig agitatorConfig = new SparkMaxConfig();
    
    private boolean agitatorsEnabled = true;

    @SuppressWarnings("removal")
    public IntakeSubsystem() {
        // Pivot Config
        pivotConfig.idleMode(IdleMode.kBrake);
        pivotConfig.smartCurrentLimit(50); // Requested limit
        pivotConfig.inverted(true); // Reversed intake pivot
        
        // REV Bus Optimizations
        pivotConfig.signals.analogPositionPeriodMs(1000);
        pivotConfig.signals.analogVelocityPeriodMs(1000);
        pivotConfig.signals.absoluteEncoderPositionPeriodMs(1000);
        pivotConfig.signals.absoluteEncoderVelocityPeriodMs(1000);
        pivotConfig.signals.busVoltagePeriodMs(100);
        pivotConfig.signals.motorTemperaturePeriodMs(100);
        
        pivotMotor.configure(pivotConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        // Roller Config
        rollerConfig.idleMode(IdleMode.kCoast);
        rollerConfig.inverted(true); // Assuming same CCW direction for deploy/intake relative to pivot
        rollerConfig.smartCurrentLimit(50); // Lowered to 50A
        
        // Fast Ramp Rate (0s) to essentially snap to speed for pickup responsiveness
        rollerConfig.openLoopRampRate(0.1);
        rollerConfig.closedLoopRampRate(0.0);
        
        // PID for RPM control
        rollerConfig.closedLoop.pid(0.0001, 0, 0); // Very small kP to rely mostly on FeedForward
        rollerConfig.closedLoop.velocityFF(1.0 / 5676.0); // Perfect kV for a NEO
        
        // REV Bus Optimizations for Rollers
        rollerConfig.signals.analogPositionPeriodMs(1000);
        rollerConfig.signals.analogVelocityPeriodMs(1000);
        rollerConfig.signals.absoluteEncoderPositionPeriodMs(1000);
        rollerConfig.signals.absoluteEncoderVelocityPeriodMs(1000);
        
        rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Agitator Config
        agitatorConfig.idleMode(IdleMode.kBrake);
        agitatorConfig.smartCurrentLimit(30);
        agitatorConfig.inverted(IntakeConstants.AGITATOR_INVERT);

        agitatorMotor.configure(agitatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // Pivot Methods
    public void setPivotPower(double power) {
        pivotMotor.set(power);
    }

    public void stopPivot() {
        pivotMotor.stopMotor();
    }

    public double getPivotCurrent() {
        return pivotMotor.getOutputCurrent();
    }

    @SuppressWarnings("removal")
    public void setPivotNeutralMode(boolean brake) {
        pivotConfig.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
        pivotMotor.configure(pivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // Roller Methods
    public void setRollerVoltage(double volts) {
        rollerMotor.setVoltage(volts);
    }

    /**
     * Set roller speed in RPM (at the roller, accounting for 2:1 ratio)
     */
    @SuppressWarnings("removal")
    public void setRollerRPM(double rpm) {
        double motorRPM = rpm * IntakeConstants.ROLLER_GEAR_RATIO;
        rollerController.setReference(motorRPM, ControlType.kVelocity);
    }

    public void stopRollers() {
        rollerMotor.stopMotor();
    }

    public void setAgitatorVoltage(double volts) {
        if (agitatorsEnabled) {
            agitatorMotor.setVoltage(volts);
        } else {
            agitatorMotor.setVoltage(0);
        }
    }

    public void toggleAgitators() {
        agitatorsEnabled = !agitatorsEnabled;
    }

    public void setAgitatorsEnabled(boolean enabled) {
        agitatorsEnabled = enabled;
    }

    public void stopAgitator() {
        agitatorMotor.stopMotor();
    }

    @Override
    public void periodic() {
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Agitators Enabled", agitatorsEnabled);
    }
}
