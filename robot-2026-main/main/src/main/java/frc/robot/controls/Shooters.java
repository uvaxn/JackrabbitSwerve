package frc.robot.controls;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj.Timer;
@SuppressWarnings("unused")
public class Shooters {
    private final TalonFX shooterR;

    private final TalonFX shooterL;
    private final TalonFX lowerFeed;
    private final TalonFX upperFeed;
    EaseofLife MotorMode = new EaseofLife();

    private final Timer shootTimer = new Timer();
    private boolean isShooting = false;
    private static final double FEED_DELAY = 0.5; // seconds before feed kicks in
    private boolean feedActive = false;
    // avoid duplicate instantiation
    public Shooters(TalonFX shooterR, TalonFX shooterL, TalonFX lowerFeed, TalonFX upperFeed) {
        this.shooterR  = shooterR;
        this.shooterL  = shooterL;
        this.lowerFeed = lowerFeed;
        this.upperFeed = upperFeed;

        // mirrors right motor in opposite direction
        shooterL.setControl(
            new Follower(
                shooterR.getDeviceID(),
                MotorAlignmentValue.Opposed
            )
        );
    }

    // spins up shooters and starts timer
    public void shoot() {
        MotorMode.setSpeed(shooterR, -0.8);
        shootTimer.reset();
        shootTimer.start();
        isShooting = true;
        System.out.println("Shooters: spinning up, feed in " + FEED_DELAY + "s");
    }

    // kicks in feed after deley
    public void periodic() {
        if (isShooting && !feedActive && shootTimer.hasElapsed(FEED_DELAY)) {
            MotorMode.setSpeed(lowerFeed, -0.8);
            MotorMode.setSpeed(upperFeed,  0.8);
            feedActive = true;
            System.out.println("Shooters: feed active");
        }
    }

    // stops everything
    public void stopShoot() {
        MotorMode.setSpeed(shooterR,  0);
        MotorMode.setSpeed(lowerFeed, 0);
        MotorMode.setSpeed(upperFeed, 0);
        shootTimer.stop();
        shootTimer.reset();
        isShooting = false;
        feedActive = false; // reset this too
        System.out.println("Shooters: stopped");
    }
}