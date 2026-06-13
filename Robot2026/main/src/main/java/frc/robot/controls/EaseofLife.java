package frc.robot.controls;

import java.util.Optional;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Vars;
import frc.robot.vision.CameraSubsystem;
import edu.wpi.first.networktables.DoubleSubscriber;


public class EaseofLife extends SubsystemBase {

    private final BooleanPublisher isHubActivePublisher;
    private final DoubleSubscriber alignP;
    private final DoubleSubscriber alignI;
    private final DoubleSubscriber alignD;
    private final CameraSubsystem cam;

    private final DoublePublisher distToHubPublisher;
    public final ShiftManager shifts = new ShiftManager();
    public StringPublisher autoStatePublisher;
    public EaseofLife(CameraSubsystem cameraSubsystem) {
        this.cam = cameraSubsystem;
        isHubActivePublisher = NetworkTableInstance.getDefault()
            .getBooleanTopic("EaseofLife/IsHubActive")
            .publish();

        autoStatePublisher = NetworkTableInstance.getDefault()
            .getStringTopic("EaseofLife/autoState")
            .publish();
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
        nt.getDoubleTopic("EaseofLife/AlignToHubP").publish().set(Vars.AlignToHubP);
        nt.getDoubleTopic("EaseofLife/AlignToHubI").publish().set(Vars.AlignToHubI);
        nt.getDoubleTopic("EaseofLife/AlignToHubD").publish().set(Vars.AlignToHubD);



        alignP = nt.getDoubleTopic("EaseofLife/AlignToHubP").subscribe(Vars.AlignToHubP);
        alignI = nt.getDoubleTopic("EaseofLife/AlignToHubI").subscribe(Vars.AlignToHubI);
        alignD = nt.getDoubleTopic("EaseofLife/AlignToHubD").subscribe(Vars.AlignToHubD);

        distToHubPublisher = nt.getDoubleTopic("EaseofLife/Distance to Hub").publish();
        distToHubPublisher.set(0);
    }
    public void setAutoState(String state) {
        autoStatePublisher.set(state);
    }

    public void teleopInit() {
        shifts.teleopInit();
    }

    @Override
    public void periodic() {
        isHubActivePublisher.set(isHubActive());
        shifts.periodic();
        distToHubPublisher.set(cam.getDistanceToHub());
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
    public double getAlignP() { return alignP.get(); }
    public double getAlignI() { return alignI.get(); }
    public double getAlignD() { return alignD.get(); }
    
    public double getDistToHub() {
        return cam.getDistanceToHub();
    }
    public void setDistToHub() {
        distToHubPublisher.set(cam.getDistanceToHub());
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