package frc.robot.controls;
 
import java.util.Optional;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.nt;
import frc.robot.vision.CameraSubsystem;
 
public class EaseofLife extends SubsystemBase {
 
    private final CameraSubsystem cam;
    public final ShiftManager shifts = new ShiftManager();
 
    public EaseofLife(CameraSubsystem cameraSubsystem) {
        this.cam = cameraSubsystem;
        // publish initial PID values to dashboard
        nt.putP(nt.getAlignP());
        nt.putI(nt.getAlignI());
        nt.putD(nt.getAlignD());
        nt.putDisttoHub(0);
    }
 
    public void teleopInit() {
        shifts.teleopInit();
    }
 
    @Override
    public void periodic() {
        nt.isHubActive(isHubActive());
        shifts.periodic();
        nt.putDisttoHub(cam.getDistanceToHub());
    }
 
    public void setSpeed(TalonFX motor, double output) {
        motor.set(MathUtil.clamp(output, -1.0, 1.0));
    }
 
    // one request object, reused for every call to avoid allocating on every periodic() tick
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
 
    // closed-loop velocity control, in rotations per second.
    // motor's Slot0Configs (kP/kI/kD/kS/kV) must already be applied see Shooters constructor.
    public void setVelocity(TalonFX motor, double targetRotationsPerSecond) {
        motor.setControl(velocityRequest.withVelocity(targetRotationsPerSecond));
    }
 
    public void setBrake(TalonFX motor, boolean brake) {
        motor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }
 
    // These now read live values from dashboard
    public double getAlignP() { return nt.getAlignP(); }
    public double getAlignI() { return nt.getAlignI(); }
    public double getAlignD() { return nt.getAlignD(); }
 
    public double getDistToHub() { return cam.getDistanceToHub(); }
 
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
