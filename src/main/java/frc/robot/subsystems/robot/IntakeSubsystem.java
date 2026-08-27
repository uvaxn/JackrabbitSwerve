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

    private double DROP_SPEED      = 0.15;
    private double LIFT_SPEED      = 0.15;

    private static final double INTAKE_COLLECT_SPEED = -0.75; // collecting from ground

    private final CoastOut    coastOut    = new CoastOut();
    private final StaticBrake staticBrake = new StaticBrake();

    // Neither of these is a fixed constant anymore -- both get OVERWRITTEN with whatever the
    // encoder actually reads the moment their sensor first confirms active, because the
    // mechanism physically travels a bit past each sensor's activation point, so trusting an
    // assumed round number for either one would be slightly wrong on the real robot.
    // topPositionDegrees starts at the boot ASSUMPTION (0 deg, arm starts at top), then gets
    // replaced with the real measured value the first time the top sensor actually confirms.
    private double topPositionDegrees = 0.0;
    private double bottomPositionDegrees = 0.0; // overwritten for real the first time isAtBottom() trips

    private static final double GEAR_REDUCTION = 50.0; // 50 motor rotations : 1 arm rotation

    // Brought forward from the most recently tuned MotorGains.INTAKE_DROP (kP, kI, kD, kS, kV,
    // kA, kG) -- this snapshot predates that file existing, so there's nothing here to diff
    // against, but these are the real values, not new guesses. ONLY used for agitation's Motion
    // Magic below; the plain .set() travel above doesn't touch these at all.
    private static final double kP = 1.0;
    private static final double kI = 0.0;
    private static final double kD = 0.2;
    private static final double kS = 0.6;
    private static final double kV = 1.0;
    private static final double kA = 0.0;
    private static final double kG = 1.2;

    private static final double AGITATE_UP_DEGREES = 20.0; // how far above the recorded bottom to agitate
    private static final double AGITATE_TOLERANCE_ROTATIONS = 0.01; // ~3.6 degrees, "close enough" to flip direction
    // The speed knob for agitation specifically -- separate from kV/kA above (those shape the
    // profile, this caps top speed). public static + not final so it's a one-line edit between
    // matches, same spirit as DROP_SPEED/LIFT_SPEED below. Only takes effect on the NEXT
    // startAgitation() call, not instantly mid-agitation -- see configureDropMotor().
    public static double AGITATE_SPEED_ROT_PER_SEC = 2.0; // mechanism rotations per second

    private final MotionMagicExpoVoltage agitateRequest = new MotionMagicExpoVoltage(0).withSlot(0);

    private enum DropState { IDLE, MOVING_DOWN, MOVING_UP, AGITATE_UP, AGITATE_DOWN, RETURN_TO_BOTTOM }

    private DropState state = DropState.IDLE;
    // what "Seeded" means in these variables is basically has it set off the top sensor (it stores that) 
    private boolean hasSeededTop    = true; // the nail is somewhat bent at the top, so it wont set off the top sensor too well. Best to leave this true.
    // same thing except for it's the bottom sensor
    private boolean hasSeededBottom = false;

    // startAgitation() was called while NOT at the bottom yet: drive down first (normal .set()
    // path, same as a plain requestDown() would), and only actually start agitating once
    // MOVING_DOWN naturally reaches isAtBottom(). Checked in the MOVING_DOWN case below.
    private boolean startAgitationOnceAtBottom = false;

    public IntakeSubsystem(TalonFX intakeMotor, TalonFX dropMotor,
                        DigitalInput upperSensor, DigitalInput lowerSensor, EaseofLife EaseOfLife, DriveInputs DriveInputs) {
        this.intakeMotor = intakeMotor;
        this.dropMotor   = dropMotor;
        this.upperSensor = upperSensor;
        this.lowerSensor = lowerSensor;
        this.MotorMode = EaseOfLife;     

        configureDropMotor();
        // Assume the arm starts at the top (0 deg) and DON'T move it to confirm that -- it
        // stays wherever it physically is until autonomous or the driver commands it down.
        dropMotor.setPosition(degreesToMechanismRotations(topPositionDegrees));
        dropMotor.setControl(staticBrake);

    }

    // Only needed for agitation's Motion Magic -- .set() below doesn't need any config at all,
    // same as it never did. GEAR_REDUCTION makes getPosition()/getClosedLoopError() report
    // mechanism (arm) rotations directly, which is what the degrees conversion below assumes.
    private void configureDropMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;
        config.Slot0.kG = kG;

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

    /** Current tracked arm position in degrees. Top is ~0, down is negative (matches the sign
     *  .set(-DROP_SPEED)/.set(LIFT_SPEED) below already drive the raw encoder in). */
    public double getPositionDegrees() {
        return mechanismRotationsToDegrees(dropMotor.getPosition().getValueAsDouble());
    }

    /** True once Motion Magic's closed loop error says the current one-shot target (agitation
     *  leg, or the return-to-bottom command) has been reached. */
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

    /** Starts agitating. Bottom sensor (not encoder position) decides whether we're already
     *  down: if so, agitation starts immediately; if not, this drives down first via the
     *  normal MOVING_DOWN path and only starts agitating once that actually reaches bottom. */
    public void startAgitation() {
        MotionMagicConfigs mmConfig = new MotionMagicConfigs();
        mmConfig.MotionMagicCruiseVelocity = AGITATE_SPEED_ROT_PER_SEC;
        dropMotor.getConfigurator().apply(mmConfig);

        if (isAtBottom()) {
            startAgitateUp();
        } else {
            startAgitationOnceAtBottom = true;
            requestDown();
        }
    }

    /** Does NOT stop in place. Commands one Motion Magic move back to the recorded bottom and
     *  switches to RETURN_TO_BOTTOM -- periodic() flips to IDLE (still actively holding that
     *  same command, Phoenix 6 keeps executing it) once it actually arrives. */
    public void stopAgitation() {
        startAgitationOnceAtBottom = false; // in case stop lands during the pending-drive-down phase
        state = DropState.RETURN_TO_BOTTOM;
        dropMotor.setControl(
            agitateRequest.withPosition(degreesToMechanismRotations(bottomPositionDegrees)));
    }

    private void startAgitateUp() {
        state = DropState.AGITATE_UP;

        dropMotor.setControl(
            agitateRequest.withPosition(
                degreesToMechanismRotations(bottomPositionDegrees + AGITATE_UP_DEGREES)
            )
        );
    }

    private void startAgitateDown() {
        state = DropState.AGITATE_DOWN;

        dropMotor.setControl(
            agitateRequest.withPosition(
                degreesToMechanismRotations(bottomPositionDegrees)
            )
        );
    }
    /** @return true when the arm is not moving useful for command isFinished() checks */
    public boolean isIdle() {
        return state == DropState.IDLE;
    }

    /** @return true when arm is fully down and intake is collecting -- also true while
     *  agitating or returning from agitation, since that's still fundamentally "collecting",
     *  just with the extra wiggle (or settling back down after it). */
    public boolean isCollecting() {
        return (state == DropState.IDLE && isAtBottom())
            || state == DropState.AGITATE_UP
            || state == DropState.AGITATE_DOWN
            || state == DropState.RETURN_TO_BOTTOM;
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
        
    }
    public void stop() {
        MotorMode.stop(intakeMotor);
        Variables.clearSpeedLimit("intake");
    }
 

    @Override
    public void periodic() {
        double posDegrees = getPositionDegrees();

        if (isAtTop() && !hasSeededTop) {
            // Record wherever we actually are, NOT a fixed 0 -- the mechanism travels a little
            // past the sensor's activation point, so the true top isn't exactly the boot
            // assumption above.
            topPositionDegrees = posDegrees;
            hasSeededTop    = true;
            hasSeededBottom = false;
            
        }
        if (isAtBottom() && !hasSeededBottom) {
            bottomPositionDegrees = posDegrees; // record wherever we are
            hasSeededBottom = true;
            hasSeededTop    = false;
        }
        
        switch (state) {
            case MOVING_DOWN -> {
                if (isAtBottom()) {
                    if (startAgitationOnceAtBottom) {
                        startAgitationOnceAtBottom = false;
                        startAgitateUp(); // bottomPositionDegrees was just re-recorded above, same loop
                    } else {
                        dropMotor.setControl(coastOut);
                        state = DropState.IDLE;
                    }
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
                startAgitateDown();
            }
        }

        case AGITATE_DOWN -> {
            if (atAgitationGoal()) {
                startAgitateUp();
            }
        }

        case RETURN_TO_BOTTOM -> {
            if (atAgitationGoal()) {
                state = DropState.IDLE; // keeps holding the same already-issued command, now settled
            }
        }
            case IDLE -> {}
        }
    }
}