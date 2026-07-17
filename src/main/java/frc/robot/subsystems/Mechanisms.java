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
        /** Spins the shooter up. Feed mode (FH vs Regular) is set separately via
     *  {@link #FullHopperMode()} / {@link #RegularMode()} and never restarts the shooter. */
    public void StartShooting() {
        NetworkTables.putRobotState("SPINNING UP");
        shooterReady = false;
        shooters.start();
    }

    /** Switches feed behavior to Full-Hopper (continuous intake bounce */
    public void FullHopperMode() {
        NetworkTables.putRobotState("FH FIRING!");
        isFHOn = true;
        isROn = false;
        shooterReady = false; // re-arm so periodic() re-fires this mode's entry action
    }

    /** Switches feed behavior to Regular (single intake lift) without touching the shooter. */
    public void RegularMode() {
        NetworkTables.putRobotState("R FIRING!");
        isROn = true;
        isFHOn = false;
        shooterReady = false; // re-arm so periodic() re-fires this mode's entry action
    }

    /** spin up and go straight to Full-Hopper mode, purely for auto. */
    public void FullHopperShoot() {
        StartShooting();
        FullHopperMode();
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
        isIntakeOn = false;
    }

    public void StopShoot() {
        NetworkTables.putRobotState("STOPPED FIRING");
        shooters.stop();
        feeds.stop();
        intakes.stop();

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
