package frc.robot.controls;
import java.util.Optional;
import java.util.concurrent.Flow.Publisher;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
public class EaseofLife {
    // public StructPublisher<Boolean> IsHubActive;
    private BooleanPublisher isHubActivePublisher;

    // Add constructor
    public EaseofLife() {
        isHubActivePublisher = NetworkTableInstance.getDefault()
            .getBooleanTopic("IsHubActive")
            .publish();
    }

    // Add periodic to call from RobotContainer or Robot.java


    
    public void setSpeed(TalonFX motor, double output) {
        motor.set(MathUtil.clamp(output, -1.0, 1.0));
    }

    public void setVelocity() {

    }
    public void setBrake(TalonFX motor, boolean brake) {
        motor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }
    public boolean isSensorTripped(DigitalInput sensor) {
        return !sensor.get();
    }
    public boolean isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        // If we have no alliance, we cannot be enabled, therefore no hub.
            if (alliance.isEmpty()) {
                return false;
            }
            // Hub is always enabled in autonomous.
            if (DriverStation.isAutonomousEnabled()) {
                return true;
            }
            // At this point, if we're not teleop enabled, there is no hub.
            if (!DriverStation.isTeleopEnabled()) {
                return false;
            }

            // We're teleop enabled, compute.
            double matchTime = DriverStation.getMatchTime();
            String gameData = DriverStation.getGameSpecificMessage();
            // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
            if (gameData.isEmpty()) {
                return true;
            }
            boolean redInactiveFirst = false;
            switch (gameData.charAt(0)) {
                case 'R' -> redInactiveFirst = true;
                case 'B' -> redInactiveFirst = false;
                default -> {
                // If we have invalid game data, assume hub is active.
                return true;
                }
            }

            // Shift was is active for blue if red won auto, or red if blue won auto.
            boolean shift1Active = switch (alliance.get()) {
                case Red -> !redInactiveFirst;
                case Blue -> redInactiveFirst;
            };

            if (matchTime > 130) {
                // Transition shift, hub is active.
                return true;
            } else if (matchTime > 105) {
                // Shift 1
                return shift1Active;
            } else if (matchTime > 80) {
                // Shift 2
                return !shift1Active;
            } else if (matchTime > 55) {
                // Shift 3
                return shift1Active;
            } else if (matchTime > 30) {
                // Shift 4
                return !shift1Active;
            } else {
                // End game, hub always active.
                return true;
            }
    }
    public void periodic() {
        isHubActivePublisher.set(isHubActive());
    }
}