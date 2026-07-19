package frc.robot.subsystems.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The intake drop arm. Sweeps from 0 degrees (up, stowed) down to 45 degrees (down, deployed)
 * using CTRE's Motion Magic Expo, the same style of generator Tuner X produces for an arm
 * mechanism, instead of the old open loop timed move.
 *
 * IMPORTANT, read before trusting any number in this file:
 *
 * 1. GEAR_REDUCTION below is set to 1.0 (direct drive) because that is what was asked for, but
 *    this is very likely wrong. The old code capped the drop motor's raw encoder position at
 *    50 rotations, and the old code could hold the arm up at the top using nothing but
 *    StaticBrake (no active current). A true 1:1 direct drive Kraken cannot do either of those
 *    things while holding up a ~15-20 lb arm at ~10-13 inches, the math below (kG_VOLTS) comes
 *    out to about 19 V, more than the ~12 V the robot actually has. That mismatch is the
 *    system telling you GEAR_REDUCTION is wrong, not a bug in this file. Measure the real
 *    ratio (spin the arm by hand end to end while watching rotor rotations in Tuner X, or read
 *    it off the gearbox) and fix the constant, the constructor below will also print a
 *    DriverStation warning at startup if kG looks physically impossible.
 *
 * 2. CTRE's Arm_Cosine gravity feedforward assumes a raw closed loop position of 0 rotations
 *    equals horizontal. Our 0 degrees is "stowed/up", which is almost certainly not horizontal
 *    on the real robot. UP_ANGLE_ABOVE_HORIZONTAL_DEGREES exists to correct for that, it is
 *    left at 0 (meaning "assume up is horizontal") since the real geometry isn't in the code
 *    anywhere. Measure it and fix the constant.
 *
 * 3. The two DigitalInput hard sensors are wired to the RoboRIO, not to the TalonFX itself, so
 *    they can't be plugged into TalonFXConfiguration's HardwareLimitSwitch feature (that needs
 *    the switch wired directly into the Talon). This file gets the same safety benefit in
 *    software instead: periodic() always trusts a tripped sensor over whatever Motion Magic
 *    thinks the position is, and re-zeroes off of it.
 */
public class IntakeDropSubsystem extends SubsystemBase {

    private final TalonFX dropMotor;
    private final DigitalInput topSensor;
    private final DigitalInput bottomSensor;

    private final MotionMagicExpoVoltage motionMagicRequest = new MotionMagicExpoVoltage(0).withSlot(0);
    private final CoastOut coastOut = new CoastOut();
    private final StaticBrake staticBrake = new StaticBrake();

    // ======================= Sweep, in the arm's own frame =======================
    private static final double UP_POSITION_DEGREES   = 0.0;
    private static final double DOWN_POSITION_DEGREES = 45.0;

    // See note 2 above. 0 means "assume up is horizontal", almost certainly wrong.
    private static final double UP_ANGLE_ABOVE_HORIZONTAL_DEGREES = 0.0; // TODO verify on robot

    // ======================= Gearing, see note 1 above =======================
    private static final double GEAR_REDUCTION = 1.0; // TODO verify, probably NOT really 1:1

    // ======================= Arm physical estimate =======================
    // Only used to derive a starting kG/kV/kA, not measured off the real robot.
    private static final double ARM_LENGTH_METERS = Units.inchesToMeters(11.5); // ~10-13 in estimate
    private static final double ARM_MASS_KG       = Units.lbsToKilograms(17.5); // ~15-20 lb estimate
    private static final double ARM_COM_METERS    = ARM_LENGTH_METERS / 2.0;    // assumes a uniform arm
    private static final double ARM_MOI_KGM2       = (ARM_MASS_KG * ARM_LENGTH_METERS * ARM_LENGTH_METERS) / 3.0; // uniform rod about one end

    // Kraken X60 datasheet numbers at 12 V. Swap these two if this is actually a Falcon 500 or
    // Kraken X44.
    private static final double MOTOR_STALL_TORQUE_NM = 7.09;
    private static final double MOTOR_FREE_SPEED_RPS  = 100.0;

    private static final double GRAVITY_TORQUE_NM = ARM_MASS_KG * 9.81 * ARM_COM_METERS;

    // Derived starting gains, in "mechanism" units since Feedback.SensorToMechanismRatio below
    // is set to GEAR_REDUCTION. Treat these as a first guess to load in and iterate on with
    // Tuner X, not a final answer, kS especially can't be derived on paper.
    private static final double kG_VOLTS          = (GRAVITY_TORQUE_NM / GEAR_REDUCTION) / MOTOR_STALL_TORQUE_NM * 12.0;
    private static final double kV_VOLTS_PER_RPS  = 12.0 * GEAR_REDUCTION / MOTOR_FREE_SPEED_RPS;
    private static final double kA_VOLTS_PER_RPS2 = (ARM_MOI_KGM2 * 2.0 * Math.PI / GEAR_REDUCTION) * (12.0 / MOTOR_STALL_TORQUE_NM);
    private static final double kS_VOLTS          = 0.25; // static friction, tune by hand on the robot

    private static final double POSITION_TOLERANCE_ROTATIONS = 0.01; // ~3.6 degrees at the mechanism

    private enum DropState { IDLE_UP, IDLE_DOWN, MOVING_UP, MOVING_DOWN }
    private DropState state = DropState.IDLE_UP;

    // what "Seeded" means here is has it set off the relevant hard sensor and had its
    // position re-zeroed off of it, same idea as the old code.
    private boolean hasSeededTop    = true; // matches the old code's note: the top sensor is unreliable, default true
    private boolean hasSeededBottom = false;

    // ---------------- bounce (continuous up/down cycling while shooting) ----------------
    private final Timer bounceTimer = new Timer();
    private static final double BOUNCE_UP_TIME = 0.6; // time held at top
    private enum BounceState { OFF, GOING_UP, GOING_DOWN }
    private BounceState bounceState = BounceState.OFF;
    private boolean bouncing = false;

    public IntakeDropSubsystem(TalonFX dropMotor, DigitalInput topSensor, DigitalInput bottomSensor) {
        this.dropMotor = dropMotor;
        this.topSensor = topSensor;
        this.bottomSensor = bottomSensor;

        if (kG_VOLTS > 6.0) {
            DriverStation.reportWarning(
                "IntakeDropSubsystem: derived kG of " + kG_VOLTS + " V (using GEAR_REDUCTION = "
                + GEAR_REDUCTION + ") is more than half the available bus voltage. "
                + "GEAR_REDUCTION is almost certainly wrong, fix it before trusting Motion Magic here.",
                false);
        }

        configureDropMotor();
        dropMotor.setPosition(userDegreesToMechanismRotations(UP_POSITION_DEGREES));
        dropMotor.setControl(staticBrake);
    }

    private void configureDropMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kS = kS_VOLTS;
        config.Slot0.kV = kV_VOLTS_PER_RPS;
        config.Slot0.kA = kA_VOLTS_PER_RPS2;
        config.Slot0.kG = kG_VOLTS;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot0.kP = 60.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.5;

        config.Feedback.SensorToMechanismRatio = GEAR_REDUCTION;

        // Motion Magic Expo ignores MotionMagicAcceleration/MotionMagicJerk, kV/kA do that job
        // instead. Cruise velocity is capped well below the theoretical max (12V / kV) as a
        // conservative default for first bring up, raise it (or set to 0 for uncapped) once
        // this is tuned.
        config.MotionMagic.MotionMagicCruiseVelocity = 0.75; // mechanism rotations/sec
        config.MotionMagic.MotionMagicExpo_kV = kV_VOLTS_PER_RPS;
        config.MotionMagic.MotionMagicExpo_kA = kA_VOLTS_PER_RPS2;

        config.CurrentLimits.StatorCurrentLimit = 40.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 30.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int attempt = 0; attempt < 5 && !status.isOK(); attempt++) {
            status = dropMotor.getConfigurator().apply(config);
        }
        if (!status.isOK()) {
            DriverStation.reportWarning(
                "IntakeDrop motor " + dropMotor.getDeviceID() + " failed to configure: " + status,
                false);
        }
    }

    /**
     * Converts our 0 (up) - 45 (down) degree convention into the rotation count the TalonFX
     * needs. Feedback.SensorToMechanismRatio already handles the gearing, this only handles
     * the horizontal offset that Arm_Cosine needs, see note 2 in the class javadoc.
     */
    private static double userDegreesToMechanismRotations(double userDegrees) {
        double trueDegreesFromHorizontal = UP_ANGLE_ABOVE_HORIZONTAL_DEGREES - userDegrees;
        return trueDegreesFromHorizontal / 360.0;
    }

    public void requestDown() {
        if (isAtBottom() && !RobotBase.isSimulation()) return;
        state = DropState.MOVING_DOWN;
    }

    public void requestUp() {
        if (isAtTop() && !RobotBase.isSimulation()) return;
        state = DropState.MOVING_UP;
    }

    public void startBounce() {
        bouncing = true;
        bounceState = BounceState.GOING_UP;
        requestUp();
        bounceTimer.restart();
    }

    public void stopBounce() {
        bouncing = false;
        bounceState = BounceState.OFF;
        bounceTimer.stop();
        bounceTimer.reset();
    }

    private void updateBounce() {
        if (!bouncing) return;

        switch (bounceState) {
            case GOING_UP -> {
                // Keep going up until the timer expires
                if (bounceTimer.hasElapsed(BOUNCE_UP_TIME)) {
                    requestDown();
                    bounceState = BounceState.GOING_DOWN;
                }
            }
            case GOING_DOWN -> {
                // Once bottom sensor is hit, go back up
                if (isAtBottom()) {
                    requestUp();
                    bounceTimer.restart();
                    bounceState = BounceState.GOING_UP;
                }
            }
            case OFF -> {}
        }
    }

    /** @return true when the lower hard sensor has been tripped. */
    public boolean isAtBottom() { return !bottomSensor.get(); }
    /** @return true when the upper hard sensor has been tripped. */
    public boolean isAtTop()    { return !topSensor.get(); }

    /** @return true once Motion Magic's closed loop error says it has reached its current goal. */
    public boolean atGoal() {
        return Math.abs(dropMotor.getClosedLoopError().getValueAsDouble()) < POSITION_TOLERANCE_ROTATIONS;
    }

    /** @return true when the arm is not actively moving, useful for command isFinished() checks */
    public boolean isIdle() {
        return state == DropState.IDLE_UP || state == DropState.IDLE_DOWN;
    }

    /** @return true when arm is fully down and settled */
    public boolean isCollecting() {
        return state == DropState.IDLE_DOWN && isAtBottom();
    }

    /** @return true when arm is fully up and settled */
    public boolean isFullyUp() {
        return state == DropState.IDLE_UP && isAtTop();
    }

    @Override
    public void periodic() {
        if (isAtTop() && !hasSeededTop) {
            dropMotor.setPosition(userDegreesToMechanismRotations(UP_POSITION_DEGREES));
            hasSeededTop    = true;
            hasSeededBottom = false;
        }
        if (isAtBottom() && !hasSeededBottom) {
            dropMotor.setPosition(userDegreesToMechanismRotations(DOWN_POSITION_DEGREES));
            hasSeededBottom = true;
            hasSeededTop    = false;
        }

        switch (state) {
            case MOVING_DOWN -> {
                if (isAtBottom()) {
                    // hard sensor says we're there no matter what Motion Magic thinks, stop
                    // asking the motor to push further and let gravity + the hard stop hold
                    // it, same as the old code did.
                    dropMotor.setControl(coastOut);
                    state = DropState.IDLE_DOWN;
                } else {
                    dropMotor.setControl(motionMagicRequest.withPosition(userDegreesToMechanismRotations(DOWN_POSITION_DEGREES)));
                }
            }
            case MOVING_UP -> {
                if (isAtTop()) {
                    dropMotor.setControl(staticBrake);
                    state = DropState.IDLE_UP;
                } else {
                    dropMotor.setControl(motionMagicRequest.withPosition(userDegreesToMechanismRotations(UP_POSITION_DEGREES)));
                }
            }
            case IDLE_UP, IDLE_DOWN -> {}
        }
        updateBounce();
    }
}