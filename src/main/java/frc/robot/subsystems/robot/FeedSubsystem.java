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
    private boolean isFeeding = false;

    private static final double UP_DURATION = 0.3; // seconds the arm goes up per bounce

    private enum FeedState { WAITING_AT_BOTTOM, GOING_UP, HOLDING_UP }
    private FeedState feedState = FeedState.WAITING_AT_BOTTOM;
    private final Timer upTimer = new Timer();

    public FeedSubsystem(IntakeSubsystem v_1, EaseofLife v_2, TalonFX m_1, TalonFX m_2) {
        this.intake = v_1;
        this.easeOfLife = v_2;
        this.lowerFeed = m_1;
        this.upperFeed = m_2;
    }

    public void start() {
        isFeeding = true;
        feedState = FeedState.WAITING_AT_BOTTOM;

        easeOfLife.setSpeed(lowerFeed, -Vars.FEED_SPEED);
        easeOfLife.setSpeed(upperFeed, Vars.FEED_SPEED);
    }

    public void stop() {
        isFeeding = false;
        easeOfLife.setSpeed(lowerFeed, 0);
        easeOfLife.setSpeed(upperFeed, 0);

        if (!intake.isAtBottom() || intake.isAtTop()) return;
        intake.requestDown();
    }

    public void periodic() {
        if (!isFeeding) return; 

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