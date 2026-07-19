package frc.robot.subsystems;
 
import java.util.Optional;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
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
import frc.robot.vision.Limelight;
 
public class EaseofLife extends SubsystemBase {
 
    private final Limelight cam;
    public final ShiftManager shifts = new ShiftManager();
    
    NetworkTable table = NetworkTableInstance.getDefault().getTable(Constants.LL_NAME);
    StructSubscriber<Pose2d> poseSubscriber = table.getStructTopic("EstimatedPose", Pose2d.struct).subscribe(new Pose2d());
    public EaseofLife(Limelight cameraSubsystem) {
        this.cam = cameraSubsystem;
        // publish initial PID values to dashboard
        NetworkTables.putAlignP(NetworkTables.getAlignP());
        NetworkTables.putAlignI(NetworkTables.getAlignI());
        NetworkTables.putAlignD(NetworkTables.getAlignD());
        NetworkTables.putAlignWallP(NetworkTables.getAlignWallP());
        NetworkTables.putAlignWallI(NetworkTables.getAlignWallI());
        NetworkTables.putAlignWallD(NetworkTables.getAlignWallD());
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
    // Free spin speed used to translate a 0.0-1.0 percent into an actual velocity target.
    // Assumes direct drive (SensorToMechanismRatio = 1.0, see configureVelocityControl below)
    // on a Kraken X60/Falcon 500 class motor. If your actual free speed is different (check
    // Tuner X's self test), this is the one number to change, it scales every setSpeed() call.
    private static final double MAX_VELOCITY_ROTATIONS_PER_SECOND = 105.0;

    // one request object, reused for every call to avoid allocating on every periodic() tick
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final NeutralOut neutralRequest = new NeutralOut();

    /** closed loop velocity control, in rotations per second.
    *   motor's Slot0Configs must already be applied see Shooters constructor.
    */ 
    public void setVelocity(TalonFX motor, double targetRotationsPerSecond) {
        motor.setControl(velocityRequest.withVelocity(targetRotationsPerSecond));
    }

    /**
     * Closed loop percent speed control. Replaces the old open loop setSpeed(motor, -1.0 to 1.0).
     * The motor must already be configured for velocity control, see configureVelocityControl()
     * below (ShooterSubsystem configures its own motors the same way, this mirrors that).
     * @param motor The TalonFX motor.
     * @param output 0.0 to 1.0, a fraction of MAX_VELOCITY_ROTATIONS_PER_SECOND.
     * @param reverse true spins the motor the other way, instead of passing a negative output.
     */
    public void setSpeed(TalonFX motor, double output, boolean reverse) {
        double percent = MathUtil.clamp(output, 0.0, 1.0);
        double targetRps = percent * MAX_VELOCITY_ROTATIONS_PER_SECOND;
        motor.setControl(velocityRequest.withVelocity(reverse ? -targetRps : targetRps));
    }

    /**
     * Stops a motor with a neutral output instead of closed looping to a 0 rps target.
     * Use this instead of setSpeed(motor, 0, false): closed looping to 0 rps actively fights
     * the motor's own spin down instead of letting it coast/brake per its NeutralMode, which
     * is a needless fight against the motor and draws current for no reason.
     */
    public void stop(TalonFX motor) {
        motor.setControl(neutralRequest);
    }

    /**
     * Applies the same PIDSVA velocity config pattern the shooters use (see
     * ShooterSubsystem.configureShooter) to a direct drive TalonFX so it can be driven with
     * setSpeed() above. These are the shooter's tuned gains reused as a starting point, a
     * roller has different inertia and friction than a flywheel, retune kS/kV/kA/kP on the
     * actual mechanism rather than trusting these blindly.
     */
    public void configureVelocityControl(TalonFX motor) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kS = 0.5;
        config.Slot0.kV = 0.11;
        config.Slot0.kA = 0.01;
        config.Slot0.kP = 0.1;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;

        config.Feedback.SensorToMechanismRatio = 1.0; // direct drive

        config.CurrentLimits.StatorCurrentLimit = 80.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 70.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLowerTime = 1.0;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int attempt = 0; attempt < 5 && !status.isOK(); attempt++) {
            status = motor.getConfigurator().apply(config);
        }
        if (!status.isOK()) {
            DriverStation.reportWarning(
                "Motor " + motor.getDeviceID() + " failed to configure for velocity control: " + status,
                false);
        }
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

    //  read live values from dashboard AlignToAllianceWall
    public double getAlignWallP() { return NetworkTables.getAlignWallP(); }
    public double getAlignWallI() { return NetworkTables.getAlignWallI(); }
    public double getAlignWallD() { return NetworkTables.getAlignWallD(); }
 
    public double getDistToHub() {
        return cam.getDistanceToHub();
     }
 
    public boolean isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty())                  return false;
        if (DriverStation.isAutonomousEnabled()) return true;
        if (!DriverStation.isTeleopEnabled())    return false;
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
        return switch (shifts.getCurrentShift()) {
            case TRANSITION, ENDGAME -> true;
            case SHIFT_1, SHIFT_3    -> shift1Active;
            case SHIFT_2, SHIFT_4    -> !shift1Active;
            default                  -> true; // won't hit here unless theres a major bug (not even my fault in this case)
        };                                       
    }
}