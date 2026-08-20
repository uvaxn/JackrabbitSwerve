package frc.robot.subsystems.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.EaseofLife;
import frc.robot.Variables;
import frc.robot.constants.MotorGains;
import frc.robot.util.NetworkTables;

public class FeedSubsystem extends SubsystemBase {
    private final IntakeDropSubsystem intakeDrop;
    private final EaseofLife easeOfLife;

    private final TalonFX upperFeed;
    private final TalonFX lowerFeed;

    public FeedSubsystem(IntakeDropSubsystem intakeDrop, EaseofLife easeOfLife, TalonFX lowerFeed, TalonFX upperFeed) {
        this.intakeDrop = intakeDrop;
        this.easeOfLife = easeOfLife;
        this.lowerFeed = lowerFeed;
        this.upperFeed = upperFeed;
        easeOfLife.configureVelocityControl(lowerFeed, MotorGains.LOWER_FEED);
        easeOfLife.configureVelocityControl(upperFeed, MotorGains.UPPER_FEED);
    }

    public void start() {
        rollersStart();
        intakeDrop.startBounce();
    }
    public void rollersStart() {
        easeOfLife.setSpeed(lowerFeed, Variables.FEED_SPEED, true);
        easeOfLife.setSpeed(upperFeed, Variables.FEED_SPEED, false);
    }
    public void stop() {
        easeOfLife.stop(lowerFeed);
        easeOfLife.stop(upperFeed);
        intakeDrop.stopBounce();
        if (!intakeDrop.hasSeededBottom) {
            intakeDrop.requestDown();
        }
    }

    @Override
    public void periodic() {
        // Telemetry only, written every loop regardless of running state so the
        // dashboard shows 0 rps when stopped rather than a stale last value.
        NetworkTables.putFeedVelocityUpper(upperFeed.getVelocity().getValueAsDouble());
        NetworkTables.putFeedVelocityLower(lowerFeed.getVelocity().getValueAsDouble());
    }
}