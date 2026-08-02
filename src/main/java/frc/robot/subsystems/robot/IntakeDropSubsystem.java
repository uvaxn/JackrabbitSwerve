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
import frc.robot.constants.MotorGains;
import frc.robot.util.NetworkTables;

/**
 * The intake drop arm. Sweeps from 0 degrees (up) to 45 degrees (down) using Motion Magic
 * Expo. Gains live in MotorGains.INTAKE_DROP, tune them there.
 *
 * Still placeholders, check before trusting this on the robot:
 * - GEAR_REDUCTION: measure the real gearbox ratio. 1.0 (direct drive) is almost certainly wrong.
 * - UP_ANGLE_ABOVE_HORIZONTAL_DEGREES: measure with an angle gauge at the real up position.
 *   Arm_Cosine gravity compensation needs to know where horizontal actually is.
 * - MotorGains.INTAKE_DROP: run SysId with the arm fully built, or tune by hand, see that file.
 *
 * The top and bottom sensors are wired to the RoboRIO, not the TalonFX, so periodic() checks
 * them here instead of using a hardware limit switch config. A tripped sensor always overrides
 * whatever Motion Magic thinks the position is.
 *
 * The arm doesn't know where it is at boot, so it starts by driving up until the top sensor
 * confirms it, the same as any other move, instead of assuming it starts at the top.
 */
public class IntakeDropSubsystem extends SubsystemBase {

    private final TalonFX dropMotor;
    private final DigitalInput topSensor;
    private final DigitalInput bottomSensor;

    private final MotionMagicExpoVoltage motionMagicRequest = new MotionMagicExpoVoltage(0).withSlot(0);
    private final CoastOut coastOut = new CoastOut();
    private final StaticBrake staticBrake = new StaticBrake();

    private static final double UP_POSITION_DEGREES = 0.0;
    private static final double DOWN_POSITION_DEGREES = 45.0;
    private static final double UP_ANGLE_ABOVE_HORIZONTAL_DEGREES = 0.0; // TODO measure on robot
    private static final double GEAR_REDUCTION = 1.0; // TODO measure, probably not 1:1

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

    private enum DropState { IDLE_UP, IDLE_DOWN, MOVING_UP, MOVING_DOWN, STALLED }
    private DropState state = DropState.MOVING_UP; // homes on boot by driving to the top sensor

    private boolean hasSeededTop = false;
    private boolean hasSeededBottom = false;

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
        moveTimer.restart();
    }

    private void configureDropMotor(MotorGains.PIDSVAG gains) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0 = gains.slot0(GravityTypeValue.Arm_Cosine);
        config.Feedback.SensorToMechanismRatio = GEAR_REDUCTION;

        // Motion Magic Expo ignores MotionMagicAcceleration and MotionMagicJerk, kV and kA
        // handle that instead. Cruise velocity is capped low as a safe default for first bring
        // up, raise it (or set to 0 for uncapped) once this is tuned.
        config.MotionMagic.MotionMagicCruiseVelocity = 0.75; // mechanism rotations per second
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

    /** Converts our 0 (up) 45 (down) degree convention into the rotation count the TalonFX
     *  needs. SensorToMechanismRatio already handles gearing, this only handles the horizontal
     *  offset Arm_Cosine needs. */
    private static double userDegreesToMechanismRotations(double userDegrees) {
        return (UP_ANGLE_ABOVE_HORIZONTAL_DEGREES - userDegrees) / 360.0;
    }

    private static double mechanismRotationsToUserDegrees(double mechanismRotations) {
        return UP_ANGLE_ABOVE_HORIZONTAL_DEGREES - (mechanismRotations * 360.0);
    }

    public void requestDown() {
        if (isAtBottom() && !RobotBase.isSimulation()) return; // sim never trips sensors
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

    /** True when the lower hard sensor is tripped, debounced. */
    public boolean isAtBottom() { return bottomSensorDebouncer.calculate(!bottomSensor.get()); }
    /** True when the upper hard sensor is tripped, debounced. */
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

    /** True when the arm is fully up and settled. */
    public boolean isFullyUp() {
        return state == DropState.IDLE_UP && isAtTop();
    }

    @Override
    public void periodic() {
        NetworkTables.putIntakeDropPositionDegrees(
            mechanismRotationsToUserDegrees(dropMotor.getPosition().getValueAsDouble()));

        // Trust a tripped sensor over the motor's own model. Only re-seed once per visit so a
        // held sensor doesn't keep re-zeroing every single loop.
        if (isAtTop() && !hasSeededTop) {
            dropMotor.setPosition(userDegreesToMechanismRotations(UP_POSITION_DEGREES));
            hasSeededTop = true;
            hasSeededBottom = false;
        }
        if (isAtBottom() && !hasSeededBottom) {
            dropMotor.setPosition(userDegreesToMechanismRotations(DOWN_POSITION_DEGREES));
            hasSeededBottom = true;
            hasSeededTop = false;
        }

        switch (state) {
            case MOVING_DOWN -> {
                if (isAtBottom()) {
                    dropMotor.setControl(coastOut);
                    state = DropState.IDLE_DOWN;
                } else if (moveTimer.hasElapsed(STALL_TIMEOUT_SECONDS)) {
                    stall();
                } else {
                    dropMotor.setControl(
                        motionMagicRequest.withPosition(userDegreesToMechanismRotations(DOWN_POSITION_DEGREES)));
                }
            }
            case MOVING_UP -> {
                if (isAtTop()) {
                    dropMotor.setControl(staticBrake);
                    state = DropState.IDLE_UP;
                } else if (moveTimer.hasElapsed(STALL_TIMEOUT_SECONDS)) {
                    stall();
                } else {
                    dropMotor.setControl(
                        motionMagicRequest.withPosition(userDegreesToMechanismRotations(UP_POSITION_DEGREES)));
                }
            }
            case IDLE_UP, IDLE_DOWN, STALLED -> {}
        }
        updateBounce();
    }

    /** Called when a move runs past STALL_TIMEOUT_SECONDS without reaching its hard sensor.
     *  Stops pushing and parks in STALLED, separate from IDLE_UP/IDLE_DOWN so telemetry can
     *  tell "finished" apart from "gave up". requestUp()/requestDown() both work normally from
     *  here and will try again. */
    private void stall() {
        DriverStation.reportWarning(
            "IntakeDropSubsystem: " + state + " timed out after " + STALL_TIMEOUT_SECONDS
            + "s without reaching its hard sensor. Stopping instead of continuing to push. "
            + "Check for a jam, a bad sensor, or an unreachable target.",
            false);
        dropMotor.setControl(coastOut);
        state = DropState.STALLED;
    }
}
