package frc.robot.subsystems.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Variables;

import frc.robot.subsystems.DriveInputs;
import frc.robot.subsystems.EaseofLife;
public class IntakeSubsystem extends SubsystemBase {

    private final TalonFX intakeMotor;
    private final TalonFX dropMotor;
    private final DigitalInput upperSensor;
    private final DigitalInput lowerSensor;
    EaseofLife MotorMode;
    private final DriveInputs DriveInputs;
    private double DROP_SPEED      = 0.15;
    private double LIFT_SPEED      = 0.15;

    private static final double INTAKE_COLLECT_SPEED = -0.6; // collecting from ground

    private final CoastOut    coastOut    = new CoastOut();
    private final StaticBrake staticBrake = new StaticBrake();

    // --- Position tracking (degrees) ---
    // Top is always 0 -- the arm is assumed to start there on init. Bottom is NOT a fixed
    // number: whatever degree value the arm reads the moment the bottom sensor first trips
    // becomes the recorded bottom, and everything downstream (agitation) uses that recorded
    // value instead of a hardcoded angle. This replaces the old SOFT_LIMIT_DOWN/SOFT_LIMIT_UP
    // raw-rotation backstops entirely -- the sensors are now the only thing that stops a move.
    private static final double TOP_POSITION_DEGREES = 0.0;
    private double bottomPositionDegrees = 0.0; // overwritten for real the first time isAtBottom() trips

    private static final double GEAR_REDUCTION = 50.0; // 50 motor rotations : 1 arm rotation

    // Brought forward from the most recently tuned MotorGains.INTAKE_DROP (kP, kI, kD, kS, kV,
    // kA, kG) -- this snapshot predates that file existing, so there's nothing here to diff
    // against, but these are the real values, not new guesses. ONLY used for agitation's Motion
    // Magic below; the plain .set() travel above doesn't touch these at all.
    private static final double AGITATE_kP = 1.0;
    private static final double AGITATE_kI = 0.0;
    private static final double AGITATE_kD = 0.2;
    private static final double AGITATE_kS = 0.6;
    private static final double AGITATE_kV = 1.0;
    private static final double AGITATE_kA = 0.0;
    private static final double AGITATE_kG = 1.2;

    private static final double AGITATE_UP_DEGREES = 20.0; // how far above the learned bottom to agitate
    private static final double AGITATE_TOLERANCE_ROTATIONS = 0.01; // ~3.6 degrees, "close enough" to flip direction
    // The speed knob for agitation specifically -- separate from kV/kA above (those shape the
    // profile, this caps top speed). public static + not final so it's a one-line edit between
    // matches, same spirit as DROP_SPEED/LIFT_SPEED below. Only takes effect on the NEXT
    // startAgitation() call, not instantly mid-agitation -- see configureDropMotor().
    public static double AGITATE_SPEED_ROT_PER_SEC = 2.0; // mechanism rotations per second

    private final MotionMagicExpoVoltage agitateRequest = new MotionMagicExpoVoltage(0).withSlot(0);

    private enum DropState { IDLE, MOVING_DOWN, MOVING_UP, AGITATE_UP, AGITATE_DOWN }

    private DropState state = DropState.IDLE;
    // what "Seeded" means in these variables is basically has it set off the top sensor (it stores that) 
    private boolean hasSeededTop    = true; // the nail is somewhat bent at the top, so it wont set off the top sensor too well. Best to leave this true.
    // same thing except for it's the bottom sensor
    private boolean hasSeededBottom = false;



    public IntakeSubsystem(TalonFX intakeMotor, TalonFX dropMotor,
                        DigitalInput upperSensor, DigitalInput lowerSensor, EaseofLife EaseOfLife, DriveInputs DriveInputs) {
        this.intakeMotor = intakeMotor;
        this.dropMotor   = dropMotor;
        this.upperSensor = upperSensor;
        this.lowerSensor = lowerSensor;
        this.MotorMode = EaseOfLife;     
        this.DriveInputs = DriveInputs;

        configureDropMotor();
        dropMotor.setPosition(degreesToMechanismRotations(TOP_POSITION_DEGREES));
        dropMotor.setControl(staticBrake);

    }

    // Only needed for agitation's Motion Magic -- .set() below doesn't need any config at all,
    // same as it never did. GEAR_REDUCTION makes getPosition()/getClosedLoopError() report
    // mechanism (arm) rotations directly, which is what the degrees conversion below assumes.
    private void configureDropMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = AGITATE_kP;
        config.Slot0.kI = AGITATE_kI;
        config.Slot0.kD = AGITATE_kD;
        config.Slot0.kS = AGITATE_kS;
        config.Slot0.kV = AGITATE_kV;
        config.Slot0.kA = AGITATE_kA;
        config.Slot0.kG = AGITATE_kG;
        // Arm_Cosine assumes 0 rotations = horizontal. Here 0 = top-of-travel instead, which may
        // or may not be horizontal on the real mechanism -- unverified. Agitation only swings
        // through a small 20 degree window near the bottom though, so the cosine curve is close
        // to flat across that range regardless of exactly where true horizontal falls.
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Feedback.SensorToMechanismRatio = GEAR_REDUCTION;
        config.MotionMagic.MotionMagicCruiseVelocity = AGITATE_SPEED_ROT_PER_SEC; // baseline; startAgitation() re-applies this fresh each time

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int attempt = 0; attempt < 5 && !status.isOK(); attempt++) {
            status = dropMotor.getConfigurator().apply(config);
        }
        if (!status.isOK()) {
            DriverStation.reportWarning(
                "IntakeSubsystem: drop motor " + dropMotor.getDeviceID() + " failed to configure: " + status,
                false);
        }
    }

    private static double degreesToMechanismRotations(double degrees) {
        return degrees / 360.0;
    }

    private static double mechanismRotationsToDegrees(double rotations) {
        return rotations * 360.0;
    }

    /** Current tracked arm position in degrees. Top is 0, down is negative (matches the sign
     *  .set(-DROP_SPEED)/.set(LIFT_SPEED) below already drive the raw encoder in). */
    public double getPositionDegrees() {
        return mechanismRotationsToDegrees(dropMotor.getPosition().getValueAsDouble());
    }

    /** True once Motion Magic's closed loop error says agitation reached its current leg's
     *  target (bottom, or bottom + 20). Only meaningful during AGITATE_UP/AGITATE_DOWN. */
    private boolean atAgitationGoal() {
        return Math.abs(dropMotor.getClosedLoopError().getValueAsDouble()) < AGITATE_TOLERANCE_ROTATIONS;
    }

    public void requestDown() {
        if (isAtBottom() && !edu.wpi.first.wpilibj.RobotBase.isSimulation()) return;
        state = DropState.MOVING_DOWN;

    }
    public void requestUp() {
        if (isAtTop() && !edu.wpi.first.wpilibj.RobotBase.isSimulation()) return;
        state = DropState.MOVING_UP;

    }

    /** Starts agitating: up 20 degrees from the recorded bottom, then back down to the
     *  recorded bottom, repeating, via Motion Magic. Assumes the arm is already at (or very
     *  near) the recorded bottom when called -- it does NOT drive down to get there first,
     *  same as the old startBounce() didn't drive anywhere before its first requestUp(). */
    public void startAgitation() {
        // Re-applied here (not just once in configureDropMotor()) so editing
        // AGITATE_SPEED_ROT_PER_SEC actually takes effect on the next agitation, not just
        // whatever it happened to be when the robot booted.
        MotionMagicConfigs mmConfig = new MotionMagicConfigs();
        mmConfig.MotionMagicCruiseVelocity = AGITATE_SPEED_ROT_PER_SEC;
        dropMotor.getConfigurator().apply(mmConfig);

        state = DropState.AGITATE_UP;
        dropMotor.setControl(
            agitateRequest.withPosition(degreesToMechanismRotations(bottomPositionDegrees + AGITATE_UP_DEGREES)));
    }

    public void stopAgitation() {
        dropMotor.setControl(coastOut);
        state = DropState.IDLE;
    }

    /** @return true when the arm is not moving useful for command isFinished() checks */
    public boolean isIdle() {
        return state == DropState.IDLE;
    }

    /** @return true when arm is fully down and intake is collecting -- also true while
     *  agitating, since that's still fundamentally "collecting", just with the extra wiggle */
    public boolean isCollecting() {
        return (state == DropState.IDLE && isAtBottom())
            || state == DropState.AGITATE_UP
            || state == DropState.AGITATE_DOWN;
    }

    /** @return true when arm is fully up */
    public boolean isFullyUp() {
        return state == DropState.IDLE && isAtTop();
    }
    /** @return true when the lower sensor has been set off. */
    public boolean isAtBottom() { return !lowerSensor.get(); }
    /** @return true when the upper sensor has been set off. */
    public boolean isAtTop()    { return !upperSensor.get(); }

    public void start() {
        MotorMode.setSpeed(intakeMotor, INTAKE_COLLECT_SPEED, false);

        Variables.requestSpeedLimit("intake", 0.15);

    CommandScheduler.getInstance().schedule(DriveInputs.rumblePulse(1, 0.5, 0.1, 0.2));
        
    }
    public void stop() {
        MotorMode.stop(intakeMotor);
        Variables.clearSpeedLimit("intake");
    }
 

    @Override
    public void periodic() {
        double posDegrees = getPositionDegrees();

        if (isAtTop() && !hasSeededTop) {
            dropMotor.setPosition(degreesToMechanismRotations(TOP_POSITION_DEGREES));
            hasSeededTop    = true;
            hasSeededBottom = false;
            
        }
        if (isAtBottom() && !hasSeededBottom) {
            bottomPositionDegrees = posDegrees; // record wherever we actually are, not a guess
            hasSeededBottom = true;
            hasSeededTop    = false;
        }
        
        switch (state) {
            case MOVING_DOWN -> {
                if (isAtBottom()) {
                    dropMotor.setControl(coastOut);
                    state = DropState.IDLE;
                    
                } else {
                    dropMotor.set(-DROP_SPEED);
                }
            }
            case MOVING_UP -> {
                if (isAtTop()) {
                    dropMotor.setControl(staticBrake);
                    state = DropState.IDLE;
                    
                } else {
                    dropMotor.set(LIFT_SPEED);
                }
            }
            case AGITATE_UP -> {
                if (atAgitationGoal()) {
                    dropMotor.setControl(
                        agitateRequest.withPosition(degreesToMechanismRotations(bottomPositionDegrees)));
                    state = DropState.AGITATE_DOWN;
                }
            }
            case AGITATE_DOWN -> {
                if (atAgitationGoal()) {
                    dropMotor.setControl(
                        agitateRequest.withPosition(degreesToMechanismRotations(bottomPositionDegrees + AGITATE_UP_DEGREES)));
                    state = DropState.AGITATE_UP;
                }
            }
            case IDLE -> {}
        }
    }
}