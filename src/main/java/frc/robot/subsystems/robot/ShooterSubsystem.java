package frc.robot.subsystems.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Variables;
import frc.robot.subsystems.EaseofLife;
import frc.robot.util.NetworkTables;
import frc.robot.util.ShooterCalculation;

/**
 * Shooter + feed subsystem. Both {@code shooterR} and {@code shooterL} run their own
 * closed-loop velocity control with identical SV/PID gains (the gains below are already
 * tuned don't change them without retuning on the robot). {@code shooterL} is mounted
 * opposite {@code shooterR}, so it's driven at the negated target velocity instead of
 * being a hardware {@link com.ctre.phoenix6.controls.Follower}. Feed motors run open-loop
 * through {@link EaseofLife}.
 */
public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX shooterR;
    private final TalonFX shooterL;
    EaseofLife MotorMode;

    private boolean IS_SHOOTING = false;
    private final Timer shooterUpdateTimer = new Timer();
    private static final double SHOOTER_UPDATE_PERIOD = 0.1; // seconds

    public ShooterSubsystem(TalonFX shooterR, TalonFX shooterL, EaseofLife EaseOfLife) {
        this.shooterR  = shooterR;
        this.shooterL  = shooterL;
        this.MotorMode = EaseOfLife;
        configureShooter(shooterR, shooterL);
    }

    private void configureShooter(TalonFX right, TalonFX left) {
        TalonFXConfiguration sharedConfig = new TalonFXConfiguration();
        sharedConfig.Slot0.kS = 0.5;
        sharedConfig.Slot0.kV = 0.11;
        sharedConfig.Slot0.kA = 0.01;
        sharedConfig.Slot0.kP = 0.1;
        sharedConfig.Slot0.kI = 0.0;
        sharedConfig.Slot0.kD = 0.0;

        sharedConfig.Feedback.SensorToMechanismRatio = 1.0; // direct drive

        applyCurrentLimitsAndNeutral(sharedConfig);
        applyConfig(right, sharedConfig);
        applyConfig(left, sharedConfig);
    }

    private void applyCurrentLimitsAndNeutral(TalonFXConfiguration config) {
        config.CurrentLimits.StatorCurrentLimit = 80.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 70.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLowerTime = 1.0;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    }

    /** Applies a config with a few retries, since a single CAN frame can occasionally drop. */
    private void applyConfig(TalonFX motor, TalonFXConfiguration config) {
        StatusCode status = null;
        boolean success = false;
        for (int attempt = 0; attempt < 5 && !success; attempt++) { // attempts to apply configs 5 times, before returning an error message
            status = motor.getConfigurator().apply(config);
            success = status.isOK();
        }
        if (!success) {
            DriverStation.reportWarning(
                "Shooter motor " + motor.getDeviceID() + " failed to configure: " + status,
                false);
        }
    }

    public void start() {
        IS_SHOOTING = true;
        shooterUpdateTimer.restart();
        // check periodic() to see where the velocity gets set.
    }

    public void stop() {
        IS_SHOOTING = false;
        MotorMode.setSpeed(shooterR, 0);
        MotorMode.setSpeed(shooterL, 0);
    }

    public boolean atSpeed() {
        double currentR = shooterR.getVelocity().getValueAsDouble();
        double currentL = shooterL.getVelocity().getValueAsDouble();
        boolean rightAtSpeed = Math.abs(currentR - (-Variables.SHOOTER_SPEED)) < 2.0;
        boolean leftAtSpeed  = Math.abs(currentL - (Variables.SHOOTER_SPEED)) < 2.0;
        return rightAtSpeed && leftAtSpeed;
    }

    public void periodic() {
        if (NetworkTables.isManualShooterOverride()) {
            Variables.SHOOTER_SPEED = NetworkTables.getShooterSpeed();
        } else {
            Variables.SHOOTER_SPEED = ShooterCalculation.calculateShooterSpeed(MotorMode.getDistToHub());
        }
        NetworkTables.putTargetShooterSpeed(Variables.SHOOTER_SPEED);
        NetworkTables.putShooterSpeed(shooterR.getVelocity().getValueAsDouble());

        if (IS_SHOOTING && shooterUpdateTimer.advanceIfElapsed(SHOOTER_UPDATE_PERIOD)) {
            MotorMode.setVelocity(shooterR, -Variables.SHOOTER_SPEED);
            MotorMode.setVelocity(shooterL, Variables.SHOOTER_SPEED);
        }
    }
}