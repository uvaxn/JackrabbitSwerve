package frc.robot.subsystems.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Vars;
import frc.robot.subsystems.EaseofLife;
import frc.robot.util.NetworkTables;
import frc.robot.util.ShooterCalc;

/**
 * Shooter + feed subsystem. {@code shooterR} runs closed-loop velocity control (the
 * SV/PID gains below are already tuned -- don't change them without retuning on the
 * robot). {@code shooterL} is a hardware {@link Follower} of {@code shooterR}, spinning
 * opposite it to match the mirrored mount (also already tuned, see
 * {@link #configureShooter}). Feed motors run open-loop through {@link EaseofLife}.
 */
public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX shooterR;
    EaseofLife MotorMode;

    private boolean isShooting = false;
    private final Timer shooterUpdateTimer = new Timer();
    private static final double SHOOTER_UPDATE_PERIOD = 0.1; // seconds

    public ShooterSubsystem(TalonFX shooterR, TalonFX shooterL, EaseofLife easeOfLife) {
        this.shooterR  = shooterR;
        this.MotorMode = easeOfLife;
        configureShooter(shooterR, shooterL);
    }

    private void configureShooter(TalonFX leader, TalonFX follower) {
        TalonFXConfiguration leaderConfig = new TalonFXConfiguration();

        // SVA/PID 
        leaderConfig.Slot0.kS = 0.5;
        leaderConfig.Slot0.kV = 0.11;
        leaderConfig.Slot0.kA = 0.01;
        leaderConfig.Slot0.kP = 0.1;
        leaderConfig.Slot0.kI = 0.0;
        leaderConfig.Slot0.kD = 0.0;

        leaderConfig.Feedback.SensorToMechanismRatio = 1.0; // direct drive
        applyCurrentLimitsAndNeutral(leaderConfig);
        applyConfig(leader, leaderConfig);

        // The follower mirrors the leader's output, so it doesn't need its own PID slot --
        // just the same current limits and neutral behavior, for its own protection.
        TalonFXConfiguration followerConfig = new TalonFXConfiguration();
        applyCurrentLimitsAndNeutral(followerConfig);
        applyConfig(follower, followerConfig);

        // follower spins opposite the leader to match the mirrored mount.
        follower.setControl(new Follower(leader.getDeviceID(), MotorAlignmentValue.Opposed));
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
        isShooting = true;
        shooterUpdateTimer.restart();
        /*  MotorMode.setVelocity(shooterR, -Vars.SHOOTER_SPEED);
         *  remeber to UNCOMMENT this once you are done tuning.
         */ 
        MotorMode.setVelocity(shooterR, NetworkTables.getShooterSpeed());
    }

    public void stop() {
        isShooting = false;
        MotorMode.setSpeed(shooterR, 0);
    }
    public boolean atSpeed() {
        double current = shooterR.getVelocity().getValueAsDouble();
        return Math.abs(current - (-Vars.SHOOTER_SPEED)) < 2.0;
    }
    public void periodic() {
        Vars.SHOOTER_SPEED = ShooterCalc.calculateShooterSpeed(MotorMode.getDistToHub());
        NetworkTables.putTargetShooterSpeed(Vars.SHOOTER_SPEED);
        NetworkTables.putShooterSpeed(shooterR.getVelocity().getValueAsDouble());

        if (isShooting && shooterUpdateTimer.advanceIfElapsed(SHOOTER_UPDATE_PERIOD)) {
            MotorMode.setVelocity(shooterR, -Vars.SHOOTER_SPEED);
        }
    }
}