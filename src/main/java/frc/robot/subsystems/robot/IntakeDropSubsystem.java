package frc.robot.subsystems.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.NetworkTables;

/**
 * The intake drop arm. Sweeps from 0 degrees (up, stowed) down to 45 degrees (down, deployed)
 * using CTRE's Motion Magic Expo control mode.
 *
 * THREE THINGS BELOW ARE STILL PLACEHOLDERS. Read this before trusting the arm on the robot:
 *
 * 1. GEAR_REDUCTION is set to 1.0 (direct drive), which is almost certainly wrong for an arm
 *    carrying real weight. This number just tells the Talon "how many times does the motor spin
 *    for one spin of the arm" - it's a units conversion, not something SysId can fix for you.
 *    Get it wrong and every rotation count the Talon reports is scaled wrong, which throws off
 *    the position targets below AND anything measured afterward (including SysId). Read it off
 *    the gearbox/sprocket ratios if documented, or measure by hand: rotate the arm a known angle
 *    by hand, watch rotor rotations in Tuner X, and scale up to a full rotation.
 *
 * 2. UP_ANGLE_ABOVE_HORIZONTAL_DEGREES is set to 0 (meaning "assume up is horizontal"), which is
 *    almost certainly wrong too. CTRE's Arm_Cosine gravity feedforward needs to know where true
 *    horizontal actually is, because gravity pulls hardest there and pulls with zero torque at
 *    vertical - get this wrong and the arm will be under- or over-compensated depending on where
 *    it is in its sweep, even once kG itself is measured correctly. Measure it with an angle
 *    gauge/level held against the arm while it's sitting at its physical "up" position.
 *
 * 3. kS_VOLTS, kG_VOLTS, kV_VOLTS_PER_RPS, and kA_VOLTS_PER_RPS2 are all left at 0.0 on purpose.
 *    These should come from a real Phoenix 6 SysId characterization run (Tuner X's SysId
 *    routine, or the WPILib SysId analyzer fed by the Phoenix SignalLogger) done with the arm
 *    fully built and under real gravity load - not hand math off an estimated arm weight and
 *    length. SysId captures real friction, real inertia, and voltage sag that a paper formula
 *    can't. See the comment above those constants for how to run each test. The constructor
 *    below prints a DriverStation warning at startup if these are still all zero.
 *
 * The two DigitalInput hard sensors are wired to the RoboRIO, not to the TalonFX itself, so they
 * can't be plugged into TalonFXConfiguration's HardwareLimitSwitch feature (that needs the
 * switch wired directly into the Talon). This file gets the same safety benefit in software
 * instead: periodic() always trusts a tripped sensor over whatever Motion Magic thinks the
 * position is, and re-zeroes off of it.
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

    // How many degrees above true horizontal the "up" position sits. See note 2 in the class
    // javadoc for why Arm_Cosine needs this. 0 here means "assume up is horizontal" - a
    // placeholder, almost certainly wrong until measured.
    private static final double UP_ANGLE_ABOVE_HORIZONTAL_DEGREES = 0.0; // TODO measure on robot

    // ======================= Gearing, see note 1 in the class javadoc =======================
    private static final double GEAR_REDUCTION = 1.0; // TODO measure, probably NOT really 1:1

    // ======================= Feedforward + PID gains, see note 3 in the class javadoc =======================
    // All four below should come from a Phoenix 6 SysId run, done with the arm fully built:
    //   kS - open-loop voltage control, slowly ramp up from 0V until the arm just barely starts
    //        to move smoothly (not jump). That voltage is kS.
    //   kG - with GravityType already set to Arm_Cosine, hold the arm at exactly horizontal in
    //        open-loop voltage control and find the voltage that holds it still without
    //        drifting up or down. That voltage is kG. (SysId's fit can also report this
    //        directly if GravityType is configured before the SysId run.)
    //   kV, kA - from the SysId quasistatic (slow ramp) and dynamic (quick step) test fits, run
    //        in both directions.
    private static final double kS_VOLTS          = 0.0; // TODO measure via SysId (see above)
    private static final double kG_VOLTS          = 0.0; // TODO measure via SysId (see above)
    private static final double kV_VOLTS_PER_RPS  = 0.0; // TODO measure via SysId (see above)
    private static final double kA_VOLTS_PER_RPS2 = 0.0; // TODO measure via SysId (see above)

    private static final double POSITION_TOLERANCE_ROTATIONS = 0.01; // ~3.6 degrees at the mechanism

    // Reed/limit switches chatter right at the trigger point, and both the seed/re-zero logic
    // and every state transition below key off isAtTop()/isAtBottom(), so an undebounced read
    // could cause rapid state flips right as the arm crosses the sensor. kBoth debounces the
    // signal going true or false, 20ms is enough to filter contact bounce without meaningfully
    // delaying the actual safety cutoff.
    private static final double SENSOR_DEBOUNCE_SECONDS = 0.02;
    private final Debouncer topSensorDebouncer = new Debouncer(SENSOR_DEBOUNCE_SECONDS, Debouncer.DebounceType.kBoth);
    private final Debouncer bottomSensorDebouncer = new Debouncer(SENSOR_DEBOUNCE_SECONDS, Debouncer.DebounceType.kBoth);

    // If a move never reaches its hard sensor (jam, mechanical bind, a target that's actually
    // unreachable), stop pushing instead of commanding Motion Magic forever. 1.5s is a generous
    // margin over the ~0.3-0.5s a 45 degree sweep should take at the cruise velocity configured
    // below, tighten it once real speed is known.
    private static final double STALL_TIMEOUT_SECONDS = 3;
    private final Timer moveTimer = new Timer();

    private enum DropState { IDLE_UP, IDLE_DOWN, MOVING_UP, MOVING_DOWN, STALLED }
    private DropState state = DropState.IDLE_UP;

    // what "Seeded" means here is has it set off the relevant hard sensor and had its
    // position re-zeroed off of it, same idea as the old code.
    private boolean hasSeededTop    = true; //: the top sensor is unreliable, default true
    private boolean hasSeededBottom = false;

    // ---------------- bounce (continuous up/down cycling while shooting) ----------------
    private final Timer bounceTimer = new Timer();
    private static final double BOUNCE_UP_TIME = 0.75; // time held at top
    private enum BounceState { OFF, GOING_UP, GOING_DOWN }
    private BounceState bounceState = BounceState.OFF;
    private boolean bouncing = false;

    public IntakeDropSubsystem(TalonFX dropMotor, DigitalInput topSensor, DigitalInput bottomSensor) {
        this.dropMotor = dropMotor;
        this.topSensor = topSensor;
        this.bottomSensor = bottomSensor;

        if (kS_VOLTS == 0.0 && kG_VOLTS == 0.0 && kV_VOLTS_PER_RPS == 0.0 && kA_VOLTS_PER_RPS2 == 0.0) {
            DriverStation.reportWarning(
                "IntakeDropSubsystem: kS/kG/kV/kA are still all 0.0 placeholders, meaning the "
                + "arm has no gravity compensation or motion profile configured yet. Run SysId "
                + "and fill these in before trusting Motion "
                + "Magic on the real robot.",
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
        config.Slot0.kP = 5.0;
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

    /** Inverse of userDegreesToMechanismRotations, for turning a live measured position back
     *  into our 0 (up) - 45 (down) degree convention for telemetry. */
    private static double mechanismRotationsToUserDegrees(double mechanismRotations) {
        return UP_ANGLE_ABOVE_HORIZONTAL_DEGREES - (mechanismRotations * 360.0);
    }

    public void requestDown() {
        if (isAtBottom() && !RobotBase.isSimulation()) return;
        state = DropState.MOVING_DOWN;
        moveTimer.restart();
    }

    public void requestUp() {
        if (isAtTop() && !RobotBase.isSimulation()) return;
        state = DropState.MOVING_UP;
        moveTimer.restart();
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

    /** @return true when the lower hard sensor has been tripped, debounced. */
    public boolean isAtBottom() { return bottomSensorDebouncer.calculate(!bottomSensor.get()); }
    /** @return true when the upper hard sensor has been tripped, debounced. */
    public boolean isAtTop()    { return topSensorDebouncer.calculate(!topSensor.get()); }

    /** @return true once Motion Magic's closed loop error says it has reached its current goal. */
    public boolean atGoal() {
        return Math.abs(dropMotor.getClosedLoopError().getValueAsDouble()) < POSITION_TOLERANCE_ROTATIONS;
    }

    /** @return true when the arm is not actively moving (including a stall), useful for
     *  command isFinished() checks. */
    public boolean isIdle() {
        return state == DropState.IDLE_UP || state == DropState.IDLE_DOWN || state == DropState.STALLED;
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
        NetworkTables.putIntakeDropPositionDegrees(
            mechanismRotationsToUserDegrees(dropMotor.getPosition().getValueAsDouble()));

        if (isAtTop() && !hasSeededTop) {
            dropMotor.setPosition(userDegreesToMechanismRotations(UP_POSITION_DEGREES));
            hasSeededTop    = true;
            hasSeededBottom = false;
            // set position to be at whatever the up position in degrees is, since we can trust the sensors more than the motor.
        }
        if (isAtBottom() && !hasSeededBottom) {
            dropMotor.setPosition(userDegreesToMechanismRotations(DOWN_POSITION_DEGREES));
            hasSeededBottom = true;
            hasSeededTop    = false;
            // set position to be at whatever the down position in degrees is, since we can trust the sensors more than the motor.
        }

        switch (state) {
            case MOVING_DOWN -> {
                if (isAtBottom()) {
                    // hard sensor says we're there no matter what Motion Magic thinks, stop
                    // asking the motor to push further
                    dropMotor.setControl(coastOut);
                    state = DropState.IDLE_DOWN;
                } else if (moveTimer.hasElapsed(STALL_TIMEOUT_SECONDS)) {
                    stall();
                } else {
                    dropMotor.setControl(motionMagicRequest.withPosition(userDegreesToMechanismRotations(DOWN_POSITION_DEGREES)));
                }
            }
            case MOVING_UP -> {
                if (isAtTop()) {
                    dropMotor.setControl(staticBrake);
                    state = DropState.IDLE_UP;
                } else if (moveTimer.hasElapsed(STALL_TIMEOUT_SECONDS)) {
                    stall();
                } else {
                    dropMotor.setControl(motionMagicRequest.withPosition(userDegreesToMechanismRotations(UP_POSITION_DEGREES)));
                }
            }
            case IDLE_UP, IDLE_DOWN, STALLED -> {}
        }
        updateBounce();
    }

    /**
     * Called when a move has run past STALL_TIMEOUT_SECONDS without reaching its hard sensor.
     * Stops pushing and parks in STALLED (distinct from IDLE_UP/IDLE_DOWN on purpose, so
     * telemetry/logs can tell "finished" apart from "gave up"). requestUp()/requestDown() both
     * work normally from here and will try again.
     */
    private void stall() {
        DriverStation.reportWarning(
            "IntakeDropSubsystem: " + state + " timed out after " + STALL_TIMEOUT_SECONDS
            + "s without reaching its hard sensor, stopping instead of continuing to push. "
            + "Check for a jam, a bad sensor, or a Motion Magic target that's unreachable.",
            false);
        dropMotor.setControl(coastOut);
        state = DropState.STALLED;
    }
}