package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.robot.FeedSubsystem;
import frc.robot.subsystems.robot.IntakeSubsystem;
import frc.robot.subsystems.robot.ShooterSubsystem;
import frc.robot.util.NetworkTables;

public class Mechanisms extends SubsystemBase {
    private ShooterSubsystem shooters;
    private IntakeSubsystem intakes;
    private FeedSubsystem feeds;
    private boolean wantIntake = false;
    private boolean isIntakeOn = false;
    private boolean isFHOn = false;
    private boolean isROn = false;

    private boolean shooterReady = false;

    public Mechanisms(ShooterSubsystem ShooterSubsystem, IntakeSubsystem IntakeSubsystem, FeedSubsystem FeedSubsystem) {
        this.shooters = ShooterSubsystem;
        this.intakes = IntakeSubsystem;
        this.feeds = FeedSubsystem;
    }
    
    public void FullHopperShoot() {
        NetworkTables.putRobotState("FH FIRING!");
        isFHOn = true;
        shooters.start();
    }

    public void RegularShoot() {
        NetworkTables.putRobotState("R FIRING!");
        isROn = true;
        shooters.start();
    }
    public void Intake() {
        NetworkTables.putRobotState("INTAKE");
        wantIntake = true;  
        intakes.requestDown();
    }

    public void StopIntake() {
        NetworkTables.putRobotState("STOPPED INTAKE");
        wantIntake = false;
        intakes.stop();
        intakes.requestUp();
        isIntakeOn = false;
    }

    public void StopShoot() {
        NetworkTables.putRobotState("STOPPED FIRING");
        shooters.stop();
        if (isFHOn && !isROn) {
            feeds.stop();
        } else if (isROn && !isFHOn) {
            intakes.stop();
            feeds.stop();
        } else {
            feeds.stop();
            intakes.stop();
        }
        isFHOn = false;
        isROn = false;

        shooterReady = false;
    }

    @Override

    public void periodic() {
        if (wantIntake && intakes.isAtBottom() && !isIntakeOn) {
            intakes.start();
            isIntakeOn = true;
        }


        if (isROn && shooters.atSpeed() && !shooterReady) {
            NetworkTables.putRobotState("R SHOOTER READY");

            feeds.rollersStart();
            intakes.requestUp();
            shooterReady = true;
        }
        if  (isFHOn && shooters.atSpeed() && !shooterReady) {
            NetworkTables.putRobotState("FH SHOOTER READY");

            feeds.start();
            shooterReady = true;

        }
    }
}
