package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.robot.FeedSubsystem;
import frc.robot.subsystems.robot.IntakeSubsystem;
import frc.robot.subsystems.robot.ShooterSubsystem;
import frc.robot.util.nt;

public class Mechanisms extends SubsystemBase {
    private ShooterSubsystem shooters;
    private IntakeSubsystem intakes;
    private FeedSubsystem feeds;
    private boolean wantIntake = false;
    private boolean isIntakeOn = false;
    private boolean isFHOn = false;
    private boolean isROn = false;
    public Mechanisms(ShooterSubsystem v_1, IntakeSubsystem v_2, FeedSubsystem v_3) {
        this.shooters = v_1;
        this.intakes = v_2;
        this.feeds = v_3;
    }
    
    public void FullHopperShoot() {
        nt.putRobotState("FH SHOOTING");
        isFHOn = true;
        shooters.start();

        feeds.start();
    }

    public void RegularShoot() {
        nt.putRobotState("R SHOOTING");
        isROn = true;
        shooters.start();

        intakes.requestUp();

        feeds.rollersStart();
    }
    public void Intake() {
        nt.putRobotState("INTAKE");
        wantIntake = true;  
        intakes.requestDown();
    }

    public void StopIntake() {
        nt.putRobotState("STOPPED INTAKE");
        wantIntake = false;
        intakes.stop();
        intakes.requestUp();
        isIntakeOn = false;
    }

    public void StopShoot() {
        nt.putRobotState("STOPPED SHOOT");
        shooters.stop();
        if (isFHOn && !isROn) {
            feeds.stop();
        } else if (isROn && !isFHOn) {
            intakes.stop();
        } else {
            feeds.stop();
            intakes.stop();
        }
        isFHOn = false;
        isROn = false;
    }

    @Override
    public void periodic() {
        if (wantIntake && intakes.isAtBottom() && !isIntakeOn) {
            intakes.start();
            isIntakeOn = true;
        }
    }
}
