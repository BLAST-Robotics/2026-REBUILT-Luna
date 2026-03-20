package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.FieldConstants.Zone;
import frc.robot.util.AllianceManager;
import frc.robot.util.DistanceManager;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX rightMotor = new TalonFX(ShooterConstants.RIGHT_MOTOR_ID);
    private final TalonFX leftMotor = new TalonFX(ShooterConstants.LEFT_MOTOR_ID);
    private final SparkMax indexer = new SparkMax(ShooterConstants.INDEXER_MOTOR_ID, MotorType.kBrushless);
    private final SparkMaxConfig indexerConfig = new SparkMaxConfig();

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    private boolean indexerStarted = false;

    public static InterpolatingDoubleTreeMap shooterHubMap = new InterpolatingDoubleTreeMap();
    public static InterpolatingDoubleTreeMap shooterNZMap = new InterpolatingDoubleTreeMap();

    PIDController pidController;
    SimpleMotorFeedforward feedforward;

    static {
        shooterHubMap.put(1.0, 1000.0);
        shooterHubMap.put(2.0, 2000.0);

        shooterNZMap.put(1.0, 1000.0);
    }

    @SuppressWarnings("removal")
    public ShooterSubsystem() {
        TalonFXConfiguration rightConfig = new TalonFXConfiguration();
        rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Add current limits
        rightConfig.CurrentLimits.StatorCurrentLimit = 70.0;
        rightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rightConfig.CurrentLimits.SupplyCurrentLimit = 70.0;
        rightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        rightConfig.Slot0.kV = ShooterConstants.SHOOTER_kV;
        rightConfig.Slot0.kP = ShooterConstants.SHOOTER_kP;

        rightConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 1.0;
        rightConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 1.0;

        rightMotor.getConfigurator().apply(rightConfig);

        // Left motor follows right motor but is inverted relative to it
        leftMotor.setControl(new Follower(ShooterConstants.RIGHT_MOTOR_ID, MotorAlignmentValue.Opposed));

        // Ensure left motor starts in Coast and applies current limits
        TalonFXConfiguration leftConfig = new TalonFXConfiguration();
        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leftConfig.CurrentLimits.StatorCurrentLimit = 70.0;
        leftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        leftConfig.CurrentLimits.SupplyCurrentLimit = 70.0;
        leftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        leftMotor.getConfigurator().apply(leftConfig);

        // Config indexer (Clockwise positive, Coast mode)
        indexerConfig.inverted(true); // Clockwise positive
        indexerConfig.idleMode(IdleMode.kCoast);
        indexerConfig.smartCurrentLimit(40);
        indexerConfig.openLoopRampRate(1.0);
        indexerConfig.closedLoopRampRate(1.0);

        // REV Bus Optimizations for Indexer
        indexerConfig.signals.analogPositionPeriodMs(1000);
        indexerConfig.signals.analogVelocityPeriodMs(1000);
        indexerConfig.signals.absoluteEncoderPositionPeriodMs(1000);
        indexerConfig.signals.absoluteEncoderVelocityPeriodMs(1000);
        indexerConfig.signals.busVoltagePeriodMs(100);
        indexerConfig.signals.motorTemperaturePeriodMs(100);

        indexer.configure(indexerConfig, SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);

        // CTRE Bus Optimizations
        // High frequency for control/telemetry on leader
        rightMotor.getVelocity().setUpdateFrequency(100);
        rightMotor.getMotorVoltage().setUpdateFrequency(50);

        // Low frequency for non-essential status on leader
        rightMotor.getDeviceTemp().setUpdateFrequency(4);
        rightMotor.getSupplyVoltage().setUpdateFrequency(4);
        rightMotor.getFault_Hardware().setUpdateFrequency(4);
        rightMotor.getAcceleration().setUpdateFrequency(4);
        rightMotor.getDutyCycle().setUpdateFrequency(4);
        rightMotor.getTorqueCurrent().setUpdateFrequency(4);

        // Follower needs very little telemetry back to RIO
        leftMotor.getVelocity().setUpdateFrequency(4);
        leftMotor.getMotorVoltage().setUpdateFrequency(4);
        leftMotor.getDeviceTemp().setUpdateFrequency(4);
        leftMotor.getSupplyVoltage().setUpdateFrequency(4);
        leftMotor.getFault_Hardware().setUpdateFrequency(4);

        pidController = new PIDController(1, 0, 0);
        feedforward = new SimpleMotorFeedforward(0 /* kS, static gain */, 0 /*
                                                                             * kV, the unit of voltage -> encoder ratio
                                                                             */, 0);
    }

    /**
     * Set the shooter velocity in rotations per second.
     * 
     * @param rotationsPerSecond Target RPS
     */
    public void setSpeed(double rotationsPerSecond) {
        rightMotor.setControl(velocityRequest.withVelocity(rotationsPerSecond));

        if (!indexerStarted && rightMotor.getVelocity().getValueAsDouble() >= 0.95 * rotationsPerSecond) {
            indexerStarted = true;
        }

        if (indexerStarted) {
            indexer.set(-ShooterConstants.INDEXER_POWER);
        } else {
            indexer.set(0);
        }
    }

    /**
     * Set the shooter motor voltage directly.
     * 
     * @param volts Target voltage
     */
    public void setVoltage(double volts) {
        rightMotor.setControl(voltageRequest.withOutput(volts));
        indexer.set(-ShooterConstants.INDEXER_POWER);
    }

    /**
     * Stop the shooter motors.
     */
    public void stop() {
        rightMotor.stopMotor();
        indexer.stopMotor();
        indexerStarted = false;
    }

    /**
     * Toggle the neutral mode of the motors.
     * 
     * @param brake True for Brake mode, false for Coast mode.
     */
    public void setNeutralMode(boolean brake) {
        NeutralModeValue mode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        TalonFXConfiguration config = new TalonFXConfiguration();

        // We only really need to apply the neutral mode to both as the follower might
        // not inherit this specific setting in all P6 versions
        rightMotor.getConfigurator().refresh(config.MotorOutput);
        config.MotorOutput.NeutralMode = mode;
        rightMotor.getConfigurator().apply(config.MotorOutput);

        leftMotor.getConfigurator().refresh(config.MotorOutput);
        config.MotorOutput.NeutralMode = mode;
        leftMotor.getConfigurator().apply(config.MotorOutput);
    }

    public double getActualVelocity() {
        return rightMotor.getVelocity().getValueAsDouble();
    }

    public double getAppliedVoltage() {
        return rightMotor.getMotorVoltage().getValueAsDouble();
    }

    public boolean isUpToSpeed(double target) {
        return Math.abs(getActualVelocity() - target) < ShooterConstants.VELOCITY_THRESHOLD;
    }

    @Override
    public void periodic() {

    }

    Zone robotZone;

    public double getShooterSpeed(Pose2d robotPose) {
        if (AllianceManager.chooseFromAlliance(DistanceManager.isLeftOf(FieldConstants.blueTrenchCenters[0], robotPose),
                DistanceManager.isRightOf(FieldConstants.redTrenchCenters[0], robotPose))) {
            robotZone = Zone.AZ;
        } else if (DistanceManager.isLeftOf(FieldConstants.redTrenchCenters[0], robotPose)
                && DistanceManager.isRightOf(FieldConstants.blueTrenchCenters[0], robotPose)) {
            robotZone = Zone.NZ;
        } else {
            robotZone = Zone.EZ;
        }
        Logger.recordOutput("Shooter/AllianceZone", robotZone);
        double output = 0;
        // AZ (Alliance Zone)
        if (robotZone == Zone.AZ) {
            output = shooterHubMap.get(Math.abs(DistanceManager.getPositionDistance(robotPose,
                    AllianceManager.chooseFromAlliance(FieldConstants.hubPositionBlue,
                            FieldConstants.hubPositionRed))));
        }
        /// NZ (Neutral Zone)
        else if (robotZone == Zone.NZ) {
            output = shooterNZMap.get(Math.abs(DistanceManager.getPositionDistance(robotPose,
                    AllianceManager.chooseFromAlliance(
                            new Pose2d(FieldConstants.blueShootPos.getX(), robotPose.getY(), new Rotation2d()),
                            new Pose2d(FieldConstants.redShootPos.getX(), robotPose.getY(), new Rotation2d())))));
        }
        /// EZ (Enemy Zone)
        else if (robotZone == Zone.EZ) {
            output = shooterNZMap.get(Math.abs(DistanceManager.getPositionDistance(robotPose,
                            new Pose2d(FieldConstants.neutral.getX(), robotPose.getY(), new Rotation2d()))));
        }
        Logger.recordOutput("Shooter/OutputRPM", output);
        return output;
    }
}
