package frc.robot.subsystems;
 
import java.util.Optional;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.controls.ShiftManager;
import frc.robot.util.NetworkTables;
import frc.robot.vision.CameraSubsystem;
 
public class EaseofLife extends SubsystemBase {
 
    private final CameraSubsystem cam;
    public final ShiftManager shifts = new ShiftManager();
    
    NetworkTable table = NetworkTableInstance.getDefault().getTable(Constants.LL_NAME);
    StructSubscriber<Pose2d> poseSubscriber = table.getStructTopic("EstimatedPose", Pose2d.struct).subscribe(new Pose2d());
    public EaseofLife(CameraSubsystem cameraSubsystem) {
        this.cam = cameraSubsystem;
        // publish initial PID values to dashboard
        NetworkTables.putAlignP(NetworkTables.getAlignP());
        NetworkTables.putAlignI(NetworkTables.getAlignI());
        NetworkTables.putAlignD(NetworkTables.getAlignD());
        NetworkTables.putDisttoHub(0);
    }
 
    public void teleopInit() {
        shifts.teleopInit();
    }
 
    @Override
    public void periodic() {
        NetworkTables.isHubActive(isHubActive());
        shifts.periodic();
        NetworkTables.putDisttoHub(cam.getDistanceToHub());
    }
    /** 
     * @param motor The TalonFX motor.
     * @param output -1.0 to 1.0, A percentage for what you want hte motor to spin at.
     */
    public void setSpeed(TalonFX motor, double output) {
        //  prevents any value higher than 1.0 or lower than -1.0 from being passed into set.
        motor.set(MathUtil.clamp(output, -1.0, 1.0));
    }
 
    // one request object, reused for every call to avoid allocating on every periodic() tick
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
 
    /** closed loop velocity control, in rotations per second.
    *   motor's Slot0Configs must already be applied see Shooters constructor.
    */ 
    public void setVelocity(TalonFX motor, double targetRotationsPerSecond) {
        motor.setControl(velocityRequest.withVelocity(targetRotationsPerSecond));
    }
    /** @param brake True or False.
     * 
     * @param motor
     * @param brake
     */
    public void setBrake(TalonFX motor, boolean brake) {
        motor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }
 
    //  read live values from dashboard
    public double getAlignP() { return NetworkTables.getAlignP(); }
    public double getAlignI() { return NetworkTables.getAlignI(); }
    public double getAlignD() { return NetworkTables.getAlignD(); }
 
    public double getDistToHub() {
        return cam.getDistanceToHub();
     }
 
    public boolean isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty())                  return false;
        if (DriverStation.isAutonomousEnabled()) return true;
        if (!DriverStation.isTeleopEnabled())    return false;
        double matchTime = DriverStation.getMatchTime();
        String gameData  = DriverStation.getGameSpecificMessage();
        if (gameData.isEmpty()) return false;
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
