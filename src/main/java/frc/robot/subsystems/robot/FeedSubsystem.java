package frc.robot.subsystems.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.EaseofLife;
import frc.robot.Vars;

public class FeedSubsystem extends SubsystemBase {
    private IntakeSubsystem intake;
    private EaseofLife easeOfLife;

    private TalonFX upperFeed;
    private TalonFX lowerFeed;
    private boolean IS_FEEDING = false;

    private static final double UP_DURATION = 0.3; // seconds the arm goes up per bounce

    private enum FeedState { WAITING_AT_BOTTOM, GOING_UP, HOLDING_UP }
    private FeedState feedState = FeedState.WAITING_AT_BOTTOM;
    private final Timer upTimer = new Timer();

    public FeedSubsystem(IntakeSubsystem IntakeSubsystem, EaseofLife EaseofLife, TalonFX lowerFeed, TalonFX upperFeed) {
        this.intake = IntakeSubsystem;
        this.easeOfLife = EaseofLife;
        this.lowerFeed = lowerFeed;
        this.upperFeed = upperFeed;
    }

    public void start() {
        IS_FEEDING = true;
        feedState = FeedState.WAITING_AT_BOTTOM;

        easeOfLife.setSpeed(lowerFeed, -Vars.FEED_SPEED);
        easeOfLife.setSpeed(upperFeed, Vars.FEED_SPEED);

        
    }
    public void rollersStart() {
        easeOfLife.setSpeed(lowerFeed, -Vars.FEED_SPEED);
        easeOfLife.setSpeed(upperFeed, Vars.FEED_SPEED);
    }
    public void stop() {
        IS_FEEDING = false;
        easeOfLife.setSpeed(lowerFeed, 0);
        easeOfLife.setSpeed(upperFeed, 0);

        if (intake.isAtBottom()) return;
        intake.requestDown();
    }

    public void periodic() {
        if (!IS_FEEDING) return; 

        switch (feedState) {
            case WAITING_AT_BOTTOM -> {
                if (intake.isIdle() && intake.isAtBottom()) {
                    intake.requestUp();
                    upTimer.restart();
                    feedState = FeedState.GOING_UP;
                }
            }
            case GOING_UP -> {
                // once the timer's elapsed, cut the trip short regardless of arm position
                if (upTimer.hasElapsed(UP_DURATION)) {
                    intake.requestDown();
                    feedState = FeedState.WAITING_AT_BOTTOM;
                }
            }
            case HOLDING_UP -> {
                //unused
            }
        }
    }
}