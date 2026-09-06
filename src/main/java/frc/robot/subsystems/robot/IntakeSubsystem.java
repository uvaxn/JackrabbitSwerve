package frc.robot.subsystems.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.DriveInputs;
import frc.robot.subsystems.EaseofLife;

/**
 * Arm/roller intake subsystem.
 *
 * Pivot control adapted from WCP's Competitive Concept Intake.java pattern:
 * TalonFX + Motion Magic position control to named angles, instead of raw
 * open-loop percent output driven until a sensor trips.
 *
 * --- VERIFY BEFORE MATCH ---
 * - Motor: Kraken X60 assumed for the free-speed calc (100 rot/s at the motor).
 *   Update kMotorFreeSpeed if it's actually a Falcon 500 or something else.
 * - Gear ratio: 50 motor rotations : 1 arm rotation (as given).
 * - Slot0 kP/kV and current limits are starting points carried over from WCP's
 *   Intake pivot — they were never tuned for THIS arm's mass/length. Retune on
 *   the real mechanism before trusting it in a match.
 * - InvertedValue.CounterClockwise_Positive: verify positive output actually
 *   drives the arm DOWN (toward 100 deg). Flip to Clockwise_Positive if the
 *   first test moves it the wrong way.
 * - Sensor wiring assumed active-low (isAtTop()/isAtBottom() return !get()),
 *   matching your original code's convention.
 * - Which physical sensor is "upper" vs "lower" doesn't actually matter for
 *   correctness anymore — each one independently learns its own angle on its
 *   first trigger. The names are just carried over from the original file.
 * - DataLogManager.start() must be called once (usually in Robot.java's
 *   robotInit()) for the log entries below to actually land in a .wpilog file.
 */
public class IntakeSubsystem extends SubsystemBase {

    public enum Position {
        UP(0),
        DOWN(100);

        private final double degrees;

        private Position(double degrees) {
            this.degrees = degrees;
        }

        public Angle angle() {
            return Degrees.of(degrees);
        }
    }

    private static final double kPivotReduction = 50.0; // 50 motor turns : 1 arm turn
    private static final AngularVelocity kMotorFreeSpeed = RotationsPerSecond.of(100.0); // Kraken X60 ~6000 RPM — VERIFY
    private static final AngularVelocity kMaxPivotSpeed = kMotorFreeSpeed.div(kPivotReduction);
    private static final Angle kPositionTolerance = Degrees.of(5);

    private static final double INTAKE_COLLECT_SPEED = -0.8; // collecting from ground
    private static final double INTAKE_FEED_SPEED = -0.5; // pushing balls to shooter
    private static final double BOUNCE_UP_TIME = 0.3; // seconds held "up" during agitate

    private final TalonFX intakeMotor;
    private final TalonFX dropMotor;
    private final DigitalInput upperSensor;
    private final DigitalInput lowerSensor;
    private final EaseofLife MotorMode;

    private final MotionMagicVoltage pivotMotionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
    private Position targetPosition = Position.UP;

    private enum BounceState { OFF, GOING_UP, GOING_DOWN }
    private BounceState bounceState = BounceState.OFF;
    private boolean bouncing = false;
    private final Timer bounceTimer = new Timer();

    // --- Sensor learn/correct state ---
    // Sensors sit at unmeasured points along the travel. The first pass by a
    // sensor records ("learns") the encoder angle at that physical point; every
    // pass after that snaps the encoder back to the learned angle, correcting
    // whatever drift crept in since the last pass.
    private boolean upperSensorLearned = false;
    private boolean lowerSensorLearned = false;
    private double upperSensorAngleDeg = 0.0;
    private double lowerSensorAngleDeg = 0.0;
    private boolean lastUpperActive = false;
    private boolean lastLowerActive = false;
    private final Debouncer upperSensorDebouncer = new Debouncer(0.02, Debouncer.DebounceType.kBoth);
    private final Debouncer lowerSensorDebouncer = new Debouncer(0.02, Debouncer.DebounceType.kBoth);

    // --- Stall detection ---
    // "Trying to move but can't": we're not at the target, the mechanism is
    // barely moving, and current is high (motor is actually pushing, not just
    // idling). Debounced so a normal start-of-motion moment (briefly near-zero
    // velocity while ramping up) doesn't get flagged as a stall.
    private static final double kStallVelocityThresholdRps = 0.5; // mechanism rot/s
    private static final double kStallCurrentThresholdAmps = 40.0;
    private final Debouncer stallDebouncer = new Debouncer(0.25, Debouncer.DebounceType.kRising);
    private boolean wasStalled = false;

    // --- Data logging: writes to the RIO's .wpilog file (view later in AdvantageScope / Tuner X) ---
    private final DataLog dataLog = DataLogManager.getLog();
    private final DoubleLogEntry logCommandedAngleDeg = new DoubleLogEntry(dataLog, "/arm/commandedAngleDeg");
    private final DoubleLogEntry logMeasuredAngleDeg = new DoubleLogEntry(dataLog, "/arm/measuredAngleDeg");
    private final BooleanLogEntry logUpperSensorActive = new BooleanLogEntry(dataLog, "/arm/upperSensorActive");
    private final BooleanLogEntry logLowerSensorActive = new BooleanLogEntry(dataLog, "/arm/lowerSensorActive");
    private final DoubleLogEntry logUpperSensorLearnedDeg =
        new DoubleLogEntry(dataLog, "/arm/upperSensorLearnedAngleDeg");
    private final DoubleLogEntry logLowerSensorLearnedDeg =
        new DoubleLogEntry(dataLog, "/arm/lowerSensorLearnedAngleDeg");
    private final DoubleLogEntry logPivotCurrentAmps = new DoubleLogEntry(dataLog, "/arm/pivotSupplyCurrentAmps");
    private final DoubleLogEntry logRollerCurrentAmps = new DoubleLogEntry(dataLog, "/arm/rollerSupplyCurrentAmps");

    public IntakeSubsystem(TalonFX intakeMotor, TalonFX dropMotor,
            DigitalInput upperSensor, DigitalInput lowerSensor, EaseofLife EaseOfLife, DriveInputs DriveInputs) {
        this.intakeMotor = intakeMotor;
        this.dropMotor = dropMotor;
        this.upperSensor = upperSensor;
        this.lowerSensor = lowerSensor;
        this.MotorMode = EaseOfLife;

        configurePivotMotor();

        // The arm always starts folded at the physical zero position, so we zero
        // the encoder there directly rather than driving into a hard stop like a
        // current-spike home.
        dropMotor.setPosition(Position.UP.angle());
        set(Position.UP);

        lastUpperActive = isAtTop();
        lastLowerActive = isAtBottom();

        SmartDashboard.putData(this);
        DataLogManager.log("[Arm] Initialized at 0 deg (UP), 50:1 gear ratio.");
    }

    private void configurePivotMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(70))
                    .withSupplyCurrentLimitEnable(true)
            )
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withSensorToMechanismRatio(kPivotReduction)
            )
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(kMaxPivotSpeed)
                    .withMotionMagicAcceleration(kMaxPivotSpeed.per(Second))
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(300)
                    .withKI(0)
                    .withKD(0)
                    .withKV(12.0 / kMaxPivotSpeed.in(RotationsPerSecond))
            );

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++) {
            status = dropMotor.getConfigurator().apply(config);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            DriverStation.reportWarning("[Arm] Pivot motor config failed: " + status, false);
        }
    }

    /** Commands the arm to Motion-Magic-profile to the given named angle. */
    public void set(Position position) {
        if (position != targetPosition) {
            DataLogManager.log("[Arm] Commanded to " + position + " (" + position.angle().in(Degrees) + " deg)");
        }
        targetPosition = position;
        dropMotor.setControl(pivotMotionMagicRequest.withPosition(position.angle()));
    }

    public void requestDown() {
        set(Position.DOWN);
    }

    public void requestUp() {
        set(Position.UP);
    }

    public void startBounce() {
        bouncing = true;
        bounceState = BounceState.GOING_UP;
        MotorMode.setSpeed(intakeMotor, INTAKE_FEED_SPEED);
        requestUp();
        bounceTimer.restart();
        DataLogManager.log("[Arm] Bounce/agitate started");
    }

    public void stopBounce() {
        bouncing = false;
        bounceState = BounceState.OFF;
        MotorMode.setSpeed(intakeMotor, 0);
        bounceTimer.stop();
        bounceTimer.reset();
        requestDown();
        DataLogManager.log("[Arm] Bounce/agitate stopped");
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
                if (isNear(Position.DOWN)) {
                    requestUp();
                    bounceTimer.restart();
                    bounceState = BounceState.GOING_UP;
                }
            }
            case OFF -> {}
        }
    }

    public void start() {
        MotorMode.setSpeed(intakeMotor, INTAKE_COLLECT_SPEED);
    }

    public void stop() {
        MotorMode.setSpeed(intakeMotor, 0);
    }

    /** @return true when the arm has reached its current target and isn't agitating. */
    public boolean isIdle() {
        return !bouncing && isNear(targetPosition);
    }

    /** @return true when the arm is down at its target (intake presumed collecting). */
    public boolean isCollecting() {
        return isIdle() && targetPosition == Position.DOWN;
    }

    /** @return true when the arm is up at its target. */
    public boolean isFullyUp() {
        return isIdle() && targetPosition == Position.UP;
    }

    /** @return true when the upper calibration sensor is currently triggered. */
    public boolean isAtTop() { return !upperSensor.get(); }

    /** @return true when the lower calibration sensor is currently triggered. */
    public boolean isAtBottom() { return !lowerSensor.get(); }

    private boolean isNear(Position position) {
        return dropMotor.getPosition().getValue().isNear(position.angle(), kPositionTolerance);
    }

    private void handleUpperSensorTrigger() {
        final double measuredDeg = dropMotor.getPosition().getValue().in(Degrees);
        if (!upperSensorLearned) {
            upperSensorAngleDeg = measuredDeg;
            upperSensorLearned = true;
            DataLogManager.log(String.format("[Arm] Upper sensor LEARNED angle: %.2f deg", measuredDeg));
        } else {
            final double driftDeg = measuredDeg - upperSensorAngleDeg;
            dropMotor.setPosition(Degrees.of(upperSensorAngleDeg));
            DataLogManager.log(String.format(
                "[Arm] Upper sensor pass: measured=%.2f deg, reset to learned=%.2f deg (corrected %.2f deg drift)",
                measuredDeg, upperSensorAngleDeg, driftDeg));
        }
        logUpperSensorLearnedDeg.append(upperSensorAngleDeg);
    }

    private void handleLowerSensorTrigger() {
        final double measuredDeg = dropMotor.getPosition().getValue().in(Degrees);
        if (!lowerSensorLearned) {
            lowerSensorAngleDeg = measuredDeg;
            lowerSensorLearned = true;
            DataLogManager.log(String.format("[Arm] Lower sensor LEARNED angle: %.2f deg", measuredDeg));
        } else {
            final double driftDeg = measuredDeg - lowerSensorAngleDeg;
            dropMotor.setPosition(Degrees.of(lowerSensorAngleDeg));
            DataLogManager.log(String.format(
                "[Arm] Lower sensor pass: measured=%.2f deg, reset to learned=%.2f deg (corrected %.2f deg drift)",
                measuredDeg, lowerSensorAngleDeg, driftDeg));
        }
        logLowerSensorLearnedDeg.append(lowerSensorAngleDeg);
    }

    @Override
    public void periodic() {
        final boolean upperActive = upperSensorDebouncer.calculate(isAtTop());
        final boolean lowerActive = lowerSensorDebouncer.calculate(isAtBottom());
        double velocityRps =
            Math.abs(dropMotor.getVelocity().getValue().in(RotationsPerSecond));

        double currentAmps =
            Math.abs(dropMotor.getSupplyCurrent().getValue().in(Amps));
        boolean tryingToMove = !isNear(targetPosition);
        boolean possibleStall =
            tryingToMove
            && velocityRps < kStallVelocityThresholdRps
            && currentAmps > kStallCurrentThresholdAmps;

        boolean stalled = stallDebouncer.calculate(possibleStall);

        if (stalled && !wasStalled) {
            DataLogManager.log(String.format(
                "[Arm] stalling! Target=%.1f deg, Position=%.1f deg, Velocity=%.2f rps, Current=%.1f A",
                targetPosition.angle().in(Degrees),
                dropMotor.getPosition().getValue().in(Degrees),
                velocityRps,
                currentAmps
            ));

            DriverStation.reportWarning(
                String.format(
                    "stalling! Target=%.1f deg, Position=%.1f deg, Current=%.1f A",
                    targetPosition.angle().in(Degrees),
                    dropMotor.getPosition().getValue().in(Degrees),
                    currentAmps
                ),
                false
            );
        }

        wasStalled = stalled;
        if (upperActive && !lastUpperActive) {
            handleUpperSensorTrigger();
        }
        if (lowerActive && !lastLowerActive) {
            handleLowerSensorTrigger();
        }
        lastUpperActive = upperActive;
        lastLowerActive = lowerActive;

        updateBounce();

        logCommandedAngleDeg.append(targetPosition.angle().in(Degrees));
        logMeasuredAngleDeg.append(dropMotor.getPosition().getValue().in(Degrees));
        logUpperSensorActive.append(upperActive);
        logLowerSensorActive.append(lowerActive);
        logPivotCurrentAmps.append(dropMotor.getSupplyCurrent().getValue().in(Amps));
        logRollerCurrentAmps.append(intakeMotor.getSupplyCurrent().getValue().in(Amps));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Target Position", () -> targetPosition.name(), null);
        builder.addDoubleProperty("Commanded Angle (deg)", () -> targetPosition.angle().in(Degrees), null);
        builder.addDoubleProperty("Measured Angle (deg)", () -> dropMotor.getPosition().getValue().in(Degrees), null);
        builder.addBooleanProperty("At Target", this::isIdle, null);
        builder.addBooleanProperty("Upper Sensor Active", this::isAtTop, null);
        builder.addBooleanProperty("Lower Sensor Active", this::isAtBottom, null);
        builder.addBooleanProperty("Upper Sensor Learned", () -> upperSensorLearned, null);
        builder.addBooleanProperty("Lower Sensor Learned", () -> lowerSensorLearned, null);
        builder.addDoubleProperty("Upper Sensor Learned Angle (deg)", () -> upperSensorAngleDeg, null);
        builder.addDoubleProperty("Lower Sensor Learned Angle (deg)", () -> lowerSensorAngleDeg, null);
        builder.addDoubleProperty("Pivot Supply Current (A)", () -> dropMotor.getSupplyCurrent().getValue().in(Amps), null);
        builder.addDoubleProperty(
            "Roller Supply Current (A)", () -> intakeMotor.getSupplyCurrent().getValue().in(Amps), null);
    }
}