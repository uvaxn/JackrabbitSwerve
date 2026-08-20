package frc.robot.subsystems.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Variables;
import frc.robot.constants.MotorGains;
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
    private final Timer shooterUpdateTimer = new Timer();

    // Gates periodic() below. Without this, periodic() unconditionally recomputed and
    // re-sent a velocity target every tick from robot boot onward -- meaning stop()'s neutral
    // command got overwritten by the very next tick and the shooter never actually idled.
    // Set true by start()/startFixed(), false by stop().
    private boolean isShooting = false;

    // Distinguishes startFixed() (Variables.FIXED_SHOOTER_SPEED, ignores vision/manual-override
    // entirely) from a regular start() (vision distance calc, or the dashboard manual override).
    private boolean fixedSpeedMode = false;
    public ShooterSubsystem(TalonFX shooterR, TalonFX shooterL, EaseofLife EaseOfLife) {
        this.shooterR  = shooterR;
        this.shooterL  = shooterL;
        this.MotorMode = EaseOfLife;
        configureShooter(shooterR, shooterL);
    }

    private void configureShooter(TalonFX right, TalonFX left) {
        TalonFXConfiguration sharedConfig = new TalonFXConfiguration();
        sharedConfig.Slot0 = MotorGains.SHOOTER.slot0();

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
        isShooting = true;
        fixedSpeedMode = false;
        shooterUpdateTimer.restart();
        Variables.requestSpeedLimit("shooters", 0.45);
        MotorMode.setVelocity(shooterR, -Variables.SHOOTER_SPEED);
        MotorMode.setVelocity(shooterL, Variables.SHOOTER_SPEED);
        // periodic() continues to refresh this every tick as distance/target changes.
    }

    /**
     * Left-bumper backup shot: spins to Variables.FIXED_SHOOTER_SPEED (live-tunable, see
     * NetworkTables.getFixedShooterSpeed()) instead of the vision/distance-calculated target,
     * so a bad vision read can never affect this shot. Otherwise identical to start() -- same
     * speed-limit request, same atSpeed()/Mechanisms hookup.
     */
    public void startFixed() {
        isShooting = true;
        fixedSpeedMode = true;
        shooterUpdateTimer.restart();
        Variables.requestSpeedLimit("shooters", 0.45);
        double fixedSpeed = NetworkTables.getFixedShooterSpeed();
        MotorMode.setVelocity(shooterR, -fixedSpeed);
        MotorMode.setVelocity(shooterL, fixedSpeed);
    }

    public void stop() {
        isShooting = false;
        fixedSpeedMode = false;
        Variables.requestSpeedLimit("shooters", 0);
        MotorMode.stop(shooterR);
        MotorMode.stop(shooterL);
    }

    public boolean atSpeed() {
        double currentR = shooterR.getVelocity().getValueAsDouble();
        double currentL = shooterL.getVelocity().getValueAsDouble();
        boolean rightAtSpeed = Math.abs(currentR - (-Variables.SHOOTER_SPEED)) < 2.0;
        boolean leftAtSpeed  = Math.abs(currentL - (Variables.SHOOTER_SPEED)) < 2.0;
        return rightAtSpeed && leftAtSpeed;
    }

    public void periodic() {
        if (isShooting) {
            if (fixedSpeedMode) {
                Variables.SHOOTER_SPEED = NetworkTables.getFixedShooterSpeed();
            } else if (NetworkTables.isManualShooterOverride()) {
                Variables.SHOOTER_SPEED = NetworkTables.getManualShooterSpeed();
            } else {
                Variables.SHOOTER_SPEED = ShooterCalculation.calculateShooterSpeed(MotorMode.getDistToHub());
            }
            MotorMode.setVelocity(shooterR, -Variables.SHOOTER_SPEED);
            MotorMode.setVelocity(shooterL, Variables.SHOOTER_SPEED);
        }

        // Published unconditionally (even while idle) so the dashboard shows the real
        // coast-down after stop() instead of freezing on the last commanded value.
        NetworkTables.putTargetShooterSpeed(isShooting ? Variables.SHOOTER_SPEED : 0.0);
        NetworkTables.putShooterVelocityRight(shooterR.getVelocity().getValueAsDouble());
        NetworkTables.putShooterVelocityLeft(shooterL.getVelocity().getValueAsDouble());
    }
}