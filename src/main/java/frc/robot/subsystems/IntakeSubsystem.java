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
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.VelocityVoltage;

public class IntakeSubsystem extends SubsystemBase {
    // Pivot: SparkMax + NEO
    private final SparkMax pivotMotor = new SparkMax(IntakeConstants.PIVOT_ID, MotorType.kBrushless);
    private final SparkMaxConfig pivotConfig = new SparkMaxConfig();

    // Rollers: TalonFX + Kraken X60
    private final TalonFX rollerMotor = new TalonFX(IntakeConstants.ROLLER_ID);
    private final TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    private final VelocityVoltage rollerVelocityRequest = new VelocityVoltage(0);

    // Agitator: SparkMax + NEO
    private final SparkMax agitatorMotor = new SparkMax(IntakeConstants.AGITATOR_ID, MotorType.kBrushless);
    private final SparkMaxConfig agitatorConfig = new SparkMaxConfig();
    
    private boolean agitatorsEnabled = true;

    @SuppressWarnings("removal")
    public IntakeSubsystem() {
        // Pivot Config
        pivotConfig.idleMode(IdleMode.kCoast);
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

        // Roller Config (Kraken X60)
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollerConfig.MotorOutput.Inverted = com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive; // Assuming true invert
        rollerConfig.CurrentLimits.StatorCurrentLimit = 40.0;
        rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        
        // PID for velocity control (Kraken uses RPS internally for VelocityVoltage)
        rollerConfig.Slot0.kP = 0.11; // Basic starting P for Kraken
        rollerConfig.Slot0.kI = 0.0;
        rollerConfig.Slot0.kD = 0.0;
        rollerConfig.Slot0.kV = 0.3; // Basic starting kV for Kraken
        
        rollerMotor.getConfigurator().apply(rollerConfig);

        // Agitator Config
        agitatorConfig.idleMode(IdleMode.kCoast);
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
    public void setRollerRPM(double rpm) {
        double motorRPM = rpm * IntakeConstants.ROLLER_GEAR_RATIO;
        double rps = motorRPM / 60.0;
        rollerMotor.setControl(rollerVelocityRequest.withVelocity(rps));
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
