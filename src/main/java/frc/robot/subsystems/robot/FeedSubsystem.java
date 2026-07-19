package frc.robot.subsystems.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.EaseofLife;
import frc.robot.Variables;

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
        easeOfLife.configureVelocityControl(lowerFeed);
        easeOfLife.configureVelocityControl(upperFeed);
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
        if (!intakeDrop.isAtBottom()) {
            intakeDrop.requestDown();
        }
    }

    @Override
    public void periodic() {}
}