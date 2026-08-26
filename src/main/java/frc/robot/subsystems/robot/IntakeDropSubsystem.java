package frc.robot.subsystems.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MotorGains;
import frc.robot.util.NetworkTables;

/**
 * The intake drop arm. Sweeps from 0 degrees (down) to 120 degrees (up), measured from
 * horizontal, using Motion Magic Expo. Gains live in MotorGains.INTAKE_DROP, tune them
 * there.
 *
 * Sensor design (updated 8/25):
 * - The BOTTOM sensor sits at the DOWN end (0 degrees) and is the ONLY zeroing reference.
 *   When it trips, the motor's internal position is re-seeded to DOWN_POSITION_DEGREES (0).
 * - The TOP sensor sits at the UP end (120 degrees) but is misaligned, so it is NOT used
 *   for anything. It does not stop motion and does not re-zero. It just chills.
 * - Sensors never gate motion. Moves complete via atGoal() (closed-loop error within
 *   tolerance) with the stall timeout as a safety net. This keeps Motion Magic -- and its
 *   kG gravity feedforward -- active at all times so the arm holds position instead of
 *   backdriving. The old code dropped to staticBrake/coastOut the instant a sensor tripped,
 *   which killed kG and caused the "brake, reverse, stall" symptom on d-pad up/down.
 *
 * Gear ratio: 50 motor rotations per 1 arm rotation.
 *
 * Homing: at boot the arm doesn't know where it is, so it drives slowly DOWN (open-loop
 * VoltageOut) until the bottom sensor trips and re-zeros it to 0. If the sensor isn't
 * reached within STALL_TIMEOUT_SECONDS, it assumes it's already at down, re-zeros to 0,
 * and parks. VERIFY HOMING DIRECTION on the robot -- if it homes the wrong way, flip the
 * sign of HOMING_VOLTAGE below.
 *
 * VERIFY ON ROBOT before trusting:
 * - requestUp must physically RAISE the arm and requestDown must LOWER it. If they're
 *   reversed, flip MotorOutput.Inverted in configureDropMotor() (or negate the return of
 *   userDegreesToMechanismRotations()).
 * - 0 degrees must be HORIZONTAL so kG·cos(θ) is correct (Arm_Cosine treats mechanism
 *   position 0 as horizontal). If the arm's horizontal point is at a different user-degree,
 *   update UP_ANGLE_ABOVE_HORIZONTAL_DEGREES to that value.
 * - MotorGains.INTAKE_DROP are placeholder starting values, NOT a SysId characterization.
 */
public class IntakeDropSubsystem extends SubsystemBase {

    private final TalonFX dropMotor;
    private final DigitalInput topSensor;
    private final DigitalInput bottomSensor;

    private final MotionMagicExpoVoltage motionMagicRequest = new MotionMagicExpoVoltage(0).withSlot(0);
    private final StaticBrake staticBrake = new StaticBrake();
    // Slow open-loop drive used ONLY for boot homing toward the bottom sensor.
    // Sign = down. Flip it if the arm homes the wrong way on the real robot.
    private final VoltageOut homingVoltage = new VoltageOut(-1.0);

    // Measured directly from horizontal, so these double as the Arm_Cosine reference angles.
    // 0 = down (horizontal), 120 = up (past vertical).
    private static final double UP_POSITION_DEGREES = 120.0;
    private static final double DOWN_POSITION_DEGREES = 0.0;
    private static final double UP_ANGLE_ABOVE_HORIZONTAL_DEGREES = 0.0; // 0 degrees = horizontal
    private static final double GEAR_REDUCTION = 50.0; // 50 motor rotations : 1 arm rotation, measured on robot

    private static final double POSITION_TOLERANCE_ROTATIONS = 0.01; // about 3.6 degrees
    private static final double SENSOR_DEBOUNCE_SECONDS = 0.02;
    private static final double STALL_TIMEOUT_SECONDS = 3;
    private static final double BOUNCE_UP_TIME = 0.75; // time held at top before bouncing back down

    private final Debouncer topSensorDebouncer =
        new Debouncer(SENSOR_DEBOUNCE_SECONDS, Debouncer.DebounceType.kBoth);
    private final Debouncer bottomSensorDebouncer =
        new Debouncer(SENSOR_DEBOUNCE_SECONDS, Debouncer.DebounceType.kBoth);
    private final Timer moveTimer = new Timer();
    private final Timer bounceTimer = new Timer();

    private enum DropState { HOMING, IDLE_UP, IDLE_DOWN, MOVING_UP, MOVING_DOWN, STALLED }
    private DropState state = DropState.HOMING; // homes on boot by driving down to the bottom sensor

    public boolean hasSeededBottom = false;

    private enum BounceState { OFF, GOING_UP, GOING_DOWN }
    private BounceState bounceState = BounceState.OFF;
    private boolean bouncing = false;

    public IntakeDropSubsystem(TalonFX dropMotor, DigitalInput topSensor, DigitalInput bottomSensor) {
        this.dropMotor = dropMotor;
        this.topSensor = topSensor;
        this.bottomSensor = bottomSensor;

        MotorGains.PIDSVAG gains = MotorGains.INTAKE_DROP;
        if (gains.kS() == 0.0 && gains.kG() == 0.0 && gains.kV() == 0.0 && gains.kA() == 0.0) {
            DriverStation.reportWarning(
                "IntakeDropSubsystem: MotorGains.INTAKE_DROP is still all zero. "
                + "Run SysId or tune by hand before trusting Motion Magic on the real robot.",
                false);
        }

        configureDropMotor(gains);
        // Arm the homing drive once at boot. Phoenix 6 control requests persist until
        // changed, so periodic() does NOT re-issue this -- it just polls isAtBottom().
        dropMotor.setControl(homingVoltage);
        moveTimer.restart();
    }

    private void configureDropMotor(MotorGains.PIDSVAG gains) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0 = gains.slot0(GravityTypeValue.Arm_Cosine);
        config.Feedback.SensorToMechanismRatio = GEAR_REDUCTION;

        // Motion Magic Expo ignores MotionMagicAcceleration and MotionMagicJerk; kV and kA
        // handle that instead. Cruise velocity caps the profile's max velocity (mechanism
        // rotations per second). Raised from 0.75 to 1.5 for snappier moves at the real ratio.
        config.MotionMagic.MotionMagicCruiseVelocity = 1.5;
        config.MotionMagic.MotionMagicExpo_kV = gains.kV();
        config.MotionMagic.MotionMagicExpo_kA = gains.kA();

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

    /** Converts user degrees (0 = down/horizontal, 120 = up) into mechanism rotations for
     *  the TalonFX. SensorToMechanismRatio already handles gearing; this only handles the
     *  horizontal offset Arm_Cosine needs (0 deg = horizontal = mechanism position 0). */
    private static double userDegreesToMechanismRotations(double userDegrees) {
        return (UP_ANGLE_ABOVE_HORIZONTAL_DEGREES - userDegrees) / 360.0;
    }

    private static double mechanismRotationsToUserDegrees(double mechanismRotations) {
        return UP_ANGLE_ABOVE_HORIZONTAL_DEGREES - (mechanismRotations * 360.0);
    }

    public void requestDown() {
        // Arm Motion Magic once; periodic() then just waits for atGoal(). No re-issuing --
        // Phoenix 6 holds the target with closed-loop + kG for as long as this control stays
        // active, so the arm holds position instead of backdriving.
        dropMotor.setControl(
            motionMagicRequest.withPosition(userDegreesToMechanismRotations(DOWN_POSITION_DEGREES)));
        state = DropState.MOVING_DOWN;
        moveTimer.restart();
    }

    public void requestUp() {
        dropMotor.setControl(
            motionMagicRequest.withPosition(userDegreesToMechanismRotations(UP_POSITION_DEGREES)));
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
                if (bounceTimer.hasElapsed(BOUNCE_UP_TIME)) {
                    requestDown();
                    bounceState = BounceState.GOING_DOWN;
                }
            }
            case GOING_DOWN -> {
                if (isAtBottom()) {
                    requestUp();
                    bounceTimer.restart();
                    bounceState = BounceState.GOING_UP;
                }
            }
            case OFF -> {}
        }
    }

    /** True when the lower (down-end) hard sensor is tripped, debounced. This is the only
     *  sensor used -- it re-zeros position to DOWN_POSITION_DEGREES (0) when it trips. */
    public boolean isAtBottom() { return bottomSensorDebouncer.calculate(!bottomSensor.get()); }
    /** True when the upper (up-end) hard sensor is tripped, debounced. NOT used for control
     *  or zeroing -- kept only for diagnostics since it's misaligned. */
    public boolean isAtTop()    { return topSensorDebouncer.calculate(!topSensor.get()); }

    /** True once Motion Magic's closed loop error says it reached its current goal. */
    public boolean atGoal() {
        return Math.abs(dropMotor.getClosedLoopError().getValueAsDouble()) < POSITION_TOLERANCE_ROTATIONS;
    }

    /** True when the arm is not actively moving, including a stall. Useful for command
     *  isFinished() checks. */
    public boolean isIdle() {
        return state == DropState.IDLE_UP || state == DropState.IDLE_DOWN || state == DropState.STALLED;
    }

    /** True when the arm is fully down and settled. */
    public boolean isCollecting() {
        return state == DropState.IDLE_DOWN && isAtBottom();
    }

    /** True when the arm is fully up and settled. The up sensor is no longer used, so this
     *  is just "reached the up goal." */
    public boolean isFullyUp() {
        return state == DropState.IDLE_UP;
    }

    @Override
    public void periodic() {
        NetworkTables.putIntakeDropPositionDegrees(
            mechanismRotationsToUserDegrees(dropMotor.getPosition().getValueAsDouble()));

        // The bottom (down-end) sensor is the SOLE zeroing reference. Re-seed once per
        // visit so a held sensor doesn't keep re-zeroing every loop. The top sensor does
        // nothing here -- it's misaligned and intentionally unused.
        if (isAtBottom() && !hasSeededBottom) {
            dropMotor.setPosition(userDegreesToMechanismRotations(DOWN_POSITION_DEGREES));
            hasSeededBottom = true;
        }
        // Re-arm the latch once we leave the sensor so the next visit re-zeros again.
        if (!isAtBottom()) {
            hasSeededBottom = false;
        }

        switch (state) {
            case HOMING -> {
                // homingVoltage was armed once in the constructor; just wait for the bottom
                // sensor (or the stall fallback) -- do NOT re-issue the control every loop.
                if (isAtBottom()) {
                    // Sensor just re-zeroed us to DOWN_POSITION (0). Switch to a Motion Magic
                    // hold once so kG stays active and the arm doesn't backdrive.
                    dropMotor.setControl(
                        motionMagicRequest.withPosition(userDegreesToMechanismRotations(DOWN_POSITION_DEGREES)));
                    state = DropState.IDLE_DOWN;
                } else if (moveTimer.hasElapsed(STALL_TIMEOUT_SECONDS)) {
                    // Couldn't find the bottom sensor -- assume we're already at down, re-zero, park.
                    dropMotor.setPosition(userDegreesToMechanismRotations(DOWN_POSITION_DEGREES));
                    dropMotor.setControl(
                        motionMagicRequest.withPosition(userDegreesToMechanismRotations(DOWN_POSITION_DEGREES)));
                    state = DropState.IDLE_DOWN;
                    DriverStation.reportWarning(
                        "IntakeDropSubsystem: homing did not reach the bottom sensor within "
                        + STALL_TIMEOUT_SECONDS + "s. Assuming down and parking. "
                        + "Check the bottom sensor and the HOMING_VOLTAGE direction.", false);
                }
            }
            case MOVING_DOWN -> {
                // Motion Magic was armed once in requestDown(); just wait for it to finish.
                // Do NOT re-issue setControl -- the persisted control holds DOWN with kG.
                if (atGoal()) {
                    state = DropState.IDLE_DOWN;
                } else if (moveTimer.hasElapsed(STALL_TIMEOUT_SECONDS)) {
                    stall();
                }
            }
            case MOVING_UP -> {
                // Motion Magic was armed once in requestUp(); just wait for it to finish.
                // Do NOT re-issue setControl -- the persisted control holds UP with kG.
                if (atGoal()) {
                    state = DropState.IDLE_UP;
                } else if (moveTimer.hasElapsed(STALL_TIMEOUT_SECONDS)) {
                    stall();
                }
            }
            case IDLE_UP, IDLE_DOWN, STALLED -> {}
        }
        updateBounce();
    }

    /** Called when a move runs past STALL_TIMEOUT_SECONDS without reaching its goal. Stops
     *  pushing and parks in STALLED, holding mechanically via staticBrake. Separate from
     *  IDLE_UP/IDLE_DOWN so telemetry can tell "finished" apart from "gave up."
     *  requestUp()/requestDown() both work normally from here and will try again. */
    private void stall() {
        DriverStation.reportWarning(
            "IntakeDropSubsystem: " + state + " timed out after " + STALL_TIMEOUT_SECONDS
            + "s without reaching its goal. Holding with brake. "
            + "Check for a jam, a bad sensor, or an unreachable target.",
            false);
        dropMotor.setControl(staticBrake);
        state = DropState.STALLED;
    }
}
