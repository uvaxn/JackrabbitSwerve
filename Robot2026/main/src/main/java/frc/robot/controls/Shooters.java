package frc.robot.controls;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Vars;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.nt;
import frc.robot.vision.CameraSubsystem;

public class Shooters extends SubsystemBase {
    private final TalonFX shooterR;
    private final TalonFX lowerFeed;
    private final TalonFX upperFeed;
    EaseofLife MotorMode;
    CameraSubsystem cam;
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

        // TODO: these are placeholder starting points for the shootergains SVPID
        Slot0Configs shooterGains = new Slot0Configs();
        shooterGains.kS = 0.1;  // volts to overcome static friction, start small
        shooterGains.kV = 0.12; // volts per rotation-per-second, the main "hold speed" term
        shooterGains.kP = 0.11; // volts per rps of error, corrects sag/disturbance
        shooterGains.kI = 0.0;  // leave at 0 to start
        shooterGains.kD = 0.0;  // usually unnecessary for a flywheel
        shooterR.getConfigurator().apply(shooterGains);
    }

    // spins up shooters and starts timer
    public void shoot() {
        MotorMode.setVelocity(shooterR, -20);
        MotorMode.setSpeed(lowerFeed, -Vars.FEED_SPEED);
        MotorMode.setSpeed(upperFeed, Vars.FEED_SPEED);
        intakes.startFeeding();
        nt.putRobotState("shooting");
    }

    // stops everything
    public void stopShoot() {
        MotorMode.setSpeed(shooterR, 0);
        MotorMode.setSpeed(lowerFeed, 0);
        MotorMode.setSpeed(upperFeed, 0);
        intakes.stopFeeding();
        nt.putRobotState("stop shooting");
    }
    public void simpleShoot() { // to be removed, only use for testing when pressing down on the d-pad.
        MotorMode.setVelocity(shooterR, -20);
    }
    public double calculateShooterSpeed(double dist) {
        // TODO: ⚠️⚠️⚠️⚠️⚠️\ MEASURE these on the real shooter and adjust!!!!!!!!!!!!!!! ⚠️⚠️⚠️⚠️⚠️⚠️⚠️⚠️⚠️⚠️
        final double MIN_DIST = 0;
        final double MAX_DIST = 4.5;
        final double MIN_SPEED_RPS = 20; // placeholder
        final double MAX_SPEED_RPS = 40; // placeholder
        final double EXPONENT = 1.5; // increase for steeper curve
 
        double Distance = Math.max(MIN_DIST, Math.min(MAX_DIST, dist));
 
        double doobiescootcanoe = (Distance - MIN_DIST) / (MAX_DIST - MIN_DIST); 
        return MIN_SPEED_RPS + Math.pow(doobiescootcanoe, EXPONENT) * (MAX_SPEED_RPS - MIN_SPEED_RPS);
    }

    
    public void periodic() {
        Vars.SHOOTER_SPEED = calculateShooterSpeed(MotorMode.getDistToHub());
        nt.putTargetShooterSpeed(Vars.SHOOTER_SPEED);
        nt.putShooterSpeed(shooterR.getVelocity().getValueAsDouble());
    }
}