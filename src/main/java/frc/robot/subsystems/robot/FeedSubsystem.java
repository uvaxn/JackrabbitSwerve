package frc.robot.subsystems.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.EaseofLife;
import frc.robot.Variables;

public class FeedSubsystem extends SubsystemBase {
    private IntakeSubsystem intake;
    private EaseofLife easeOfLife;

    private TalonFX upperFeed;
    private TalonFX lowerFeed;

    public FeedSubsystem(IntakeSubsystem IntakeSubsystem, EaseofLife EaseofLife, TalonFX lowerFeed, TalonFX upperFeed) {
        this.intake = IntakeSubsystem;
        this.easeOfLife = EaseofLife;
        this.lowerFeed = lowerFeed;
        this.upperFeed = upperFeed;
    }

    public void start() {
        rollersStart();
        intake.startAgitation();
    }
    public void rollersStart() {
        easeOfLife.setSpeed(lowerFeed, Variables.FEED_SPEED, true);
        easeOfLife.setSpeed(upperFeed, Variables.FEED_SPEED, false);
    }
    public void stop() {
        easeOfLife.stop(lowerFeed);;
        easeOfLife.stop(upperFeed);;
        intake.stopAgitation();
        if (!intake.isAtBottom()) {
            intake.requestDown();
        }
    }

    @Override
    public void periodic() {}
}