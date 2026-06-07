package frc.robot.controls;

import java.util.Optional;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

public class EaseofLife {

    private final BooleanPublisher isHubActivePublisher;
    public  final ShiftManager     shifts = new ShiftManager();
    public StringPublisher autoStatePublisher;
    public EaseofLife() {
        isHubActivePublisher = NetworkTableInstance.getDefault()
            .getBooleanTopic("EaseofLife/IsHubActive")
            .publish();
        autoStatePublisher = NetworkTableInstance.getDefault()
            .getStringTopic("EaseofLife/autoState")
            .publish();
    }

    public void teleopInit() {
        shifts.teleopInit();
    }

    public void periodic() {
        isHubActivePublisher.set(isHubActive());
        shifts.periodic();
    }

    public void setSpeed(TalonFX motor, double output) {
        motor.set(MathUtil.clamp(output, -1.0, 1.0));
    }

    public void setVelocity() {}

    public void setBrake(TalonFX motor, boolean brake) {
        motor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    public boolean isSensorTripped(DigitalInput sensor) {
        return !sensor.get();
    }

    public boolean isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty())                  return false;
        if (DriverStation.isAutonomousEnabled()) return true;
        if (!DriverStation.isTeleopEnabled())    return false;

        double matchTime = DriverStation.getMatchTime();
        String gameData  = DriverStation.getGameSpecificMessage();
        if (gameData.isEmpty()) return true;

        boolean redInactiveFirst = switch (gameData.charAt(0)) {
            case 'R' -> true;
            case 'B' -> false;
            default  -> true; 
        };

        boolean shift1Active = switch (alliance.get()) {
            case Red  -> !redInactiveFirst;
            case Blue ->  redInactiveFirst;
        };

        if      (matchTime > ShiftManager.TRANSITION_END) return true;
        else if (matchTime > ShiftManager.SHIFT1_END)     return shift1Active;
        else if (matchTime > ShiftManager.SHIFT2_END)     return !shift1Active;
        else if (matchTime > ShiftManager.SHIFT3_END)     return shift1Active;
        else if (matchTime > ShiftManager.ENDGAME_START)  return !shift1Active;
        else                                               return true;
    }
}