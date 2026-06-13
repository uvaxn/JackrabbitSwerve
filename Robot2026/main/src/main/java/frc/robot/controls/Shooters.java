package frc.robot.controls;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.vision.CameraSubsystem;

public class Shooters extends SubsystemBase {
    private final TalonFX shooterR;

    private final TalonFX lowerFeed;
    private final TalonFX upperFeed;
    EaseofLife MotorMode;
    CameraSubsystem cam;
    private double SHOOTER_SPEED = 0.8;
    private static final double FEED_SPEED = 0.8;
    private final IntakeSubsystem intakes;
    // avoid duplicate instantiation
    public Shooters(TalonFX shooterR, TalonFX shooterL, TalonFX lowerFeed, TalonFX upperFeed, EaseofLife easeOfLife, CameraSubsystem camerasubsystem, IntakeSubsystem intakeSubsystem) {
        this.shooterR  = shooterR;
        this.lowerFeed = lowerFeed;
        this.upperFeed = upperFeed;
        this.MotorMode = easeOfLife;
        this.intakes = intakeSubsystem;
        this.cam = camerasubsystem; 
        shooterL.setControl(new Follower(shooterR.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    // spins up shooters and starts timer
    public void shoot() {
        MotorMode.setSpeed(shooterR, -SHOOTER_SPEED);
        MotorMode.setSpeed(lowerFeed, -FEED_SPEED);
        MotorMode.setSpeed(upperFeed, 0.8);
        intakes.startFeeding();
        MotorMode.setAutoState("shooting");
    }

    // stops everything
    public void stopShoot() {
        MotorMode.setSpeed(shooterR, 0);
        MotorMode.setSpeed(lowerFeed, 0);
        MotorMode.setSpeed(upperFeed, 0);
        intakes.stopFeeding();
        MotorMode.setAutoState("shooting stopped");
    }
    public double calculateShooterSpeed(double dist) {
        // TODO: ⚠️⚠️⚠️⚠️⚠️\ adjust this please!!!!!!!!!!!!!!! ⚠️⚠️⚠️⚠️⚠️⚠️⚠️⚠️⚠️⚠️
        final double MIN_DIST = 0;
        final double MAX_DIST = 4.5;
        final double MIN_SPEED = 0.5;
        final double MAX_SPEED = 1.0;
        final double EXPONENT = 2; // increase for steeper curve

        final double Distance = Math.max(MIN_DIST, Math.min(MAX_DIST, dist));

        double doobiescootcanoe = (Distance - MIN_DIST) / (MAX_DIST - MIN_DIST); // 0 to 1
        return MIN_SPEED + Math.pow(doobiescootcanoe, EXPONENT) * (MAX_SPEED - MIN_SPEED);
    }
    public void periodic() {
        SHOOTER_SPEED = calculateShooterSpeed(MotorMode.getDistToHub());
    }
}
