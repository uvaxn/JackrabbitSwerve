package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Variables;
import frc.robot.subsystems.robot.FeedSubsystem;
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
 * wants it down, a firing sequence wants it bouncing (see FeedSubsystem.start()). That conflict
 * exists in the old code too; it's a game-strategy call, not something an enum can resolve on
 * its own. If you want one side to win, the cheapest place to arbitrate is right here (e.g.
 * requestIntake() could refuse to drop the arm while shootState == FIRING).
 */
public class Mechanisms extends SubsystemBase {

    private enum ShootState { IDLE, SPINNING_UP, FIRING }
    private enum IntakeState { IDLE, DROPPING, COLLECTING }

    private final ShooterSubsystem shooters;
    private final IntakeSubsystem intakes;
    private final FeedSubsystem feeds;

    private ShootState shootState = ShootState.IDLE;
    private IntakeState intakeState = IntakeState.IDLE;
    // True only while the left-bumper fixed/backup shot (startShootingFixed()) is active.
    // shootState alone can't tell startShooting() and startShootingFixed() apart -- both drive
    // the same SPINNING_UP/FIRING machine below -- so this is what isShootingWithVision() uses
    // to exclude the fixed shot from auto-align. See that method for why.
    private boolean fixedShotActive = false;
    // True only while d-pad-left manual fire (startShooterAndFeeds()) is active. The normal
    // right-trigger firing path calls feeds.start(), which bounces the intake-drop arm. The
    // d-pad-left binding must NOT move the arm, so it runs the feed rollers directly via
    // feeds.rollersStart() and sets this flag so the SPINNING_UP->FIRING transition below
    // skips the bouncing feeds.start().
    private boolean manualFire = false;

    public Mechanisms(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, FeedSubsystem feedSubsystem) {
        this.shooters = shooterSubsystem;
        this.intakes = intakeSubsystem;
        this.feeds = feedSubsystem;
    }

    // ==================== requests -- call these from RobotContainer bindings ====================

    /**
     * Starts (or keeps running) the shooter. Safe to call on every trigger press; it only
     * actually (re)commands the motors on the IDLE -> SPINNING_UP edge, so tapping the trigger
     * again while already shooting doesn't reset a shot that's already in progress.
     */
    public void startShooting() {
        if (shootState == ShootState.IDLE) {
            shooters.start();
        }
        shootState = ShootState.SPINNING_UP;
        fixedShotActive = false;
        Variables.requestSpeedLimit("shooters", 0.2);
        NetworkTables.putRobotState("SPINNING UP");
    }

    /**
     * Left-bumper backup: identical state machine to startShooting() (still waits for
     * shooters.atSpeed() below before feeding), just spins the shooter to
     * Variables.FIXED_SHOOTER_SPEED instead of the vision/distance-calculated target. See
     * ShooterSubsystem.startFixed(). Also flags fixedShotActive so isShootingWithVision() below
     * excludes this from auto-align -- see that method for why.
     */
    public void startShootingFixed() {
        if (shootState == ShootState.IDLE) {
            shooters.startFixed();
        }
        shootState = ShootState.SPINNING_UP;
        fixedShotActive = true;
        Variables.requestSpeedLimit("shooters", 0.2);
        NetworkTables.putRobotState("SPINNING UP (FIXED)");
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
        fixedShotActive = false;
        manualFire = false;
        Variables.clearSpeedLimit("shooters");
        NetworkTables.putRobotState("STOPPED FIRING");
    }

    /**
     * D-pad-left manual fire: spins the shooter AND runs the feed rollers, but does NOT
     * touch the intake-drop arm -- no bounce, no requestDown. The arm only ever moves via
     * its own d-pad up/down bindings. Uses feeds.rollersStart() (rollers only) instead of
     * feeds.start() (which calls intakeDrop.startBounce()), and sets manualFire so the
     * SPINNING_UP->FIRING transition below does not also call feeds.start().
     */
    public void startShooterAndFeeds() {
        if (shootState == ShootState.IDLE) {
            shooters.start();
        }
        feeds.rollersStart();
        manualFire = true;
        shootState = ShootState.SPINNING_UP;
        Variables.requestSpeedLimit("shooters", 0.2);
        NetworkTables.putRobotState("MANUAL FIRE");
    }

    /** Companion to startShooterAndFeeds(): stops the shooter and the feed rollers, still
     * without touching the arm. */
    public void stopShooterAndFeeds() {
        shooters.stop();
        feeds.stop();
        manualFire = false;
        shootState = ShootState.IDLE;
        fixedShotActive = false;
        Variables.clearSpeedLimit("shooters");
        NetworkTables.putRobotState("STOPPED MANUAL FIRE");
    }

    /**
     * @return true whenever a firing sequence is in progress (spinning up or actively firing),
     * fixed shot or vision shot alike. General-purpose "is the shooter doing anything right
     * now" signal -- if you need to gate auto-align specifically, use isShootingWithVision()
     * below instead, not this one.
     */
    public boolean isShooting() {
        return shootState != ShootState.IDLE;
    }

    /**
     * @return true only while a VISION-based shot (right trigger, startShooting()) is spinning
     * up or firing -- false during the left-bumper fixed/backup shot (startShootingFixed()),
     * even though isShooting() above is true for both. This is what actually gates
     * auto-align-while-shooting in RobotContainer: the fixed shot exists specifically so
     * shooting still works when vision is misbehaving, so it shouldn't also trigger a
     * vision-driven heading correction -- left bumper should just shoot and nothing else move.
     */
    public boolean isShootingWithVision() {
        return shootState != ShootState.IDLE && !fixedShotActive;
    }

    public void requestIntake() {
        intakeState = IntakeState.DROPPING;
        intakes.requestDown();
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
                if (intakes.isAtBottom()) {
                    intakes.start();
                    intakeState = IntakeState.COLLECTING;
                }
            }
            case COLLECTING, IDLE -> {}
        }

        switch (shootState) {
            case SPINNING_UP -> {
                if (shooters.atSpeed()) {
                    // Only the right-trigger (non-manual) path auto-feeds with the arm bounce.
                    // The d-pad-left manual path already started the rollers itself and must
                    // not bounce the arm, so it skips feeds.start() here.
                    if (!manualFire) {
                        feeds.start();
                    }
                    shootState = ShootState.FIRING;
                    NetworkTables.putRobotState("SHOOTER READY");
                }
            }
            case IDLE, FIRING -> {}
        }
    }
}