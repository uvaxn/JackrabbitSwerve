package frc.robot.subsystems.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Vars;
import frc.robot.subsystems.EaseofLife;
import frc.robot.util.nt;
import frc.robot.vision.CameraSubsystem;

public class Shooters extends SubsystemBase {
    private final TalonFX shooterR;
    private final TalonFX lowerFeed;
    private final TalonFX upperFeed;
    EaseofLife MotorMode;
    CameraSubsystem cam;
    private final IntakeSubsystem intakes;

    private boolean isShooting = false;
    private final Timer shooterUpdateTimer = new Timer();
    private static final double SHOOTER_UPDATE_PERIOD = 0.1; // seconds

    public Shooters(TalonFX shooterR, TalonFX shooterL, TalonFX lowerFeed, TalonFX upperFeed, EaseofLife easeOfLife, CameraSubsystem camerasubsystem, IntakeSubsystem intakeSubsystem) {
        this.shooterR  = shooterR;
        this.lowerFeed = lowerFeed;
        this.upperFeed = upperFeed;
        this.MotorMode = easeOfLife;
        this.intakes = intakeSubsystem;
        this.cam = camerasubsystem;
        shooterL.setControl(new Follower(shooterR.getDeviceID(), MotorAlignmentValue.Opposed));

        Slot0Configs shooterGains = new Slot0Configs();
        shooterGains.kS = 0.5;
        shooterGains.kV = 0.11;
        shooterGains.kP = 0.1;
        shooterGains.kI = 0.01;
        shooterGains.kD = 0.0;
        shooterR.getConfigurator().apply(shooterGains);
    }

    public void shoot() {
        isShooting = true;
        shooterUpdateTimer.restart();
        /*  MotorMode.setVelocity(shooterR, -Vars.SHOOTER_SPEED);
         *  remeber to UNCOMMENT this once you are done tuning.
         */ 
        MotorMode.setVelocity(shooterR, nt.getShooterSpeed());
        MotorMode.setSpeed(lowerFeed, -Vars.FEED_SPEED);
        MotorMode.setSpeed(upperFeed, Vars.FEED_SPEED);
        intakes.startFeeding();
        nt.putRobotState("shooting");
    }

    public void stopShoot() {
        isShooting = false;
        MotorMode.setSpeed(shooterR, 0);
        MotorMode.setSpeed(lowerFeed, 0);
        MotorMode.setSpeed(upperFeed, 0);
        intakes.stopFeeding();
        nt.putRobotState("stop shooting");
    }

    public void simpleShoot() { // to be removed, only use for testing when pressing down on the d-pad.
        nt.putRobotState("shooting");
        MotorMode.setVelocity(shooterR, -Vars.SHOOTER_SPEED);
    }

    public double calculateShooterSpeed(double dist) {
        final double MIN_DIST = 0;
        final double MAX_DIST = 4.5;

        double minSpeed = nt.getShooterMinSpeed();
        double maxSpeed = nt.getShooterMaxSpeed();
        double exponent = nt.getShooterExponent();

        double distance = Math.max(MIN_DIST, Math.min(MAX_DIST, dist));
        double normalized = (distance - MIN_DIST) / (MAX_DIST - MIN_DIST);

        return minSpeed
            + Math.pow(normalized, exponent)
            * (maxSpeed - minSpeed);
    }

    public void periodic() {
        Vars.SHOOTER_SPEED = calculateShooterSpeed(MotorMode.getDistToHub());
        nt.putTargetShooterSpeed(Vars.SHOOTER_SPEED);
        nt.putShooterSpeed(shooterR.getVelocity().getValueAsDouble());

        if (isShooting && shooterUpdateTimer.advanceIfElapsed(SHOOTER_UPDATE_PERIOD)) {
            MotorMode.setVelocity(shooterR, -Vars.SHOOTER_SPEED);
        }
    }
}