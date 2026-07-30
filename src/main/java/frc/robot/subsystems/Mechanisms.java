package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Variables;
import frc.robot.subsystems.robot.FeedSubsystem;
import frc.robot.subsystems.robot.IntakeDropSubsystem;
import frc.robot.subsystems.robot.IntakeSubsystem;
import frc.robot.subsystems.robot.ShooterSubsystem;
import frc.robot.util.NetworkTables;

/**
 * Orchestrates the intake and shooter/feed mechanisms as two small, explicit state machines
 * instead of five interacting booleans (the old wantIntake / isIntakeOn / isFHOn / isROn /
 * shooterReady). Each machine only ever occupies one named state, so "what is this mechanism
 * doing right now" is a single field read instead of several flags you have to reason about
 * together -- the same pattern IntakeDropSubsystem already uses internally (DropState).
 *
 * intakeState and shootState are kept independent on purpose: the original bindings let the
 * driver hold left trigger (intake) and right trigger (shoot) at once, so this does not force
 * them to be mutually exclusive. What this refactor does NOT decide for you is that both
 * machines can still try to command IntakeDropSubsystem's arm at the same time -- ground-intake
 * wants it down, a full-hopper fire wants it bouncing. That conflict exists in the old code too;
 * it's a game-strategy call, not something an enum can resolve on its own. If you want one side
 * to win, the cheapest place to arbitrate is right here (e.g. requestIntake() could refuse to
 * drop the arm while shootState == FIRING and feedMode == FULL_HOPPER).
 */
public class Mechanisms extends SubsystemBase {

    /** Which feed behavior a firing sequence uses. Chosen once, when shooting starts. */
    public enum FeedMode { REGULAR, FULL_HOPPER }

    private enum ShootState { IDLE, SPINNING_UP, FIRING }
    private enum IntakeState { IDLE, DROPPING, COLLECTING }

    private final ShooterSubsystem shooters;
    private final IntakeSubsystem intakes;
    private final IntakeDropSubsystem intakeDrop;
    private final FeedSubsystem feeds;

    private ShootState shootState = ShootState.IDLE;
    private FeedMode feedMode = FeedMode.REGULAR;
    private IntakeState intakeState = IntakeState.IDLE;

    public Mechanisms(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem,
                       IntakeDropSubsystem intakeDropSubsystem, FeedSubsystem feedSubsystem) {
        this.shooters = shooterSubsystem;
        this.intakes = intakeSubsystem;
        this.intakeDrop = intakeDropSubsystem;
        this.feeds = feedSubsystem;
    }

    // ==================== requests -- call these from RobotContainer bindings ====================

    /**
     * Starts (or keeps running) the shooter in the given mode. Safe to call on every trigger
     * press; it only actually (re)commands the motors on the IDLE -> SPINNING_UP edge, so
     * tapping the same mode twice in a row doesn't reset a shot that's already in progress.
     */
    public void startShooting(FeedMode mode) {
        if (shootState == ShootState.IDLE) {
            shooters.start();
        }
        feedMode = mode;
        shootState = ShootState.SPINNING_UP;
        Variables.requestSpeedLimit("shooters", 0.3);
        NetworkTables.putRobotState("SPINNING UP (" + (mode == FeedMode.FULL_HOPPER ? "FH" : "R") + ")");
    }

    /**
     * Stops the shooter and feed. Deliberately does NOT touch the intake -- the old StopShoot()
     * called intakes.stop() directly without resetting the intake's own request flags, so
     * releasing the shoot trigger while still holding the intake trigger would silently kill
     * ground intake until you let go and re-pressed it. Intake now owns its own lifecycle
     * exclusively; nothing outside requestIntake()/stopIntake() touches it.
     */
    public void stopShooting() {
        shooters.stop();
        feeds.stop(); // also parks the intake-drop arm back down, see FeedSubsystem.stop()
        shootState = ShootState.IDLE;
        Variables.clearSpeedLimit("shooters");
        NetworkTables.putRobotState("STOPPED FIRING");
    }

    public void requestIntake() {
        intakeState = IntakeState.DROPPING;
        intakeDrop.requestDown();
        NetworkTables.putRobotState("INTAKE");
    }

    public void stopIntake() {
        intakes.stop();
        intakeState = IntakeState.IDLE;
        NetworkTables.putRobotState("STOPPED INTAKE");
    }

    // ==================== state machines, advanced every scheduler tick ====================

    @Override
    public void periodic() {
        switch (intakeState) {
            case DROPPING -> {
                if (intakeDrop.isAtBottom()) {
                    intakes.start();
                    intakeState = IntakeState.COLLECTING;
                }
            }
            case COLLECTING, IDLE -> {}
        }

        switch (shootState) {
            case SPINNING_UP -> {
                if (shooters.atSpeed()) {
                    if (feedMode == FeedMode.FULL_HOPPER) {
                        feeds.start();
                    } else {
                        feeds.rollersStart();
                        intakeDrop.requestUp();
                    }
                    shootState = ShootState.FIRING;
                    NetworkTables.putRobotState((feedMode == FeedMode.FULL_HOPPER ? "FH" : "R") + " SHOOTER READY");
                }
            }
            case IDLE, FIRING -> {}
        }
    }
}