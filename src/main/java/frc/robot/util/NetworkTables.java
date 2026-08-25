package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import frc.robot.Variables;

public class NetworkTables {
    private static final NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
    private static final NetworkTable rbtTable = ntInst.getTable("Robot");
    private static final NetworkTable infoTable = ntInst.getTable("Info");

    // Per-mechanism subtables under "Robot". Each mechanism owns its own NT
    // branch instead of everything being flattened under "EaseofLife".
    private static final NetworkTable shooterTable   = rbtTable.getSubTable("Shooter");
    private static final NetworkTable feedTable      = rbtTable.getSubTable("Feed");
    private static final NetworkTable intakeMotorTable = rbtTable.getSubTable("Intake");
    private static final NetworkTable intakeTable    = rbtTable.getSubTable("IntakeDrop");
    private static final NetworkTable autoAlignTable = rbtTable.getSubTable("AutoAlign");
    private static final NetworkTable alignHubTable  = autoAlignTable.getSubTable("AlignToHub");
    private static final NetworkTable alignWallTable = autoAlignTable.getSubTable("AlignToWall");

    // ---- Robot (top level) ----
    private static final StringPublisher robotState = rbtTable.getStringTopic("State").publish();

    // ---- AutoAlign ----
    private static final DoublePublisher targetAngle = autoAlignTable.getDoubleTopic("TargetAngleDeg").publish();

    // AlignToHub PID
    private static final DoublePublisher AligntoHubP = alignHubTable.getDoubleTopic("P").publish();
    private static final DoublePublisher AligntoHubI = alignHubTable.getDoubleTopic("I").publish();
    private static final DoublePublisher AligntoHubD = alignHubTable.getDoubleTopic("D").publish();
    private static final DoubleSubscriber alignPSub = alignHubTable.getDoubleTopic("P").subscribe(Variables.AlignToHubP);
    private static final DoubleSubscriber alignISub = alignHubTable.getDoubleTopic("I").subscribe(Variables.AlignToHubI);
    private static final DoubleSubscriber alignDSub = alignHubTable.getDoubleTopic("D").subscribe(Variables.AlignToHubD);

    // Lead-compensation exit speed for AlignToHub/AlignWhileShooting's shoot-on-the-move
    // aiming, see Variables.FUEL_EXIT_SPEED_MPS for what this is (and isn't).
    private static final DoublePublisher fuelExitSpeedPub =
        alignHubTable.getDoubleTopic("FuelExitSpeedMPS").publish();
    private static final DoubleSubscriber fuelExitSpeedSub =
        alignHubTable.getDoubleTopic("FuelExitSpeedMPS").subscribe(Variables.FUEL_EXIT_SPEED_MPS);

    // AlignToAllianceWall PID (kept on its own topics, see prior note: reusing
    // the AlignToHub topics here caused tuning one to silently retune the other)
    private static final DoublePublisher AligntoWallP = alignWallTable.getDoubleTopic("P").publish();
    private static final DoublePublisher AligntoWallI = alignWallTable.getDoubleTopic("I").publish();
    private static final DoublePublisher AligntoWallD = alignWallTable.getDoubleTopic("D").publish();
    private static final DoubleSubscriber alignWallPSub = alignWallTable.getDoubleTopic("P").subscribe(Variables.AlignToAllianceWallP);
    private static final DoubleSubscriber alignWallISub = alignWallTable.getDoubleTopic("I").subscribe(Variables.AlignToAllianceWallI);
    private static final DoubleSubscriber alignWallDSub = alignWallTable.getDoubleTopic("D").subscribe(Variables.AlignToAllianceWallD);

    // ---- Shooter ----
    // Telemetry only — written unconditionally every periodic() loop, so nothing
    // should ever read these back as inputs (that's what the setpoint sub below is for).
    private static final DoublePublisher shooterVelocitySetpoint = shooterTable.getDoubleTopic("VelocitySetpointRPS").publish();
    // shooterR and shooterL are separate physical motors driven at negated targets
    // of each other (see ShooterSubsystem), so they get separate actual-velocity
    // topics rather than being collapsed into one "shooter speed" number.
    private static final DoublePublisher shooterVelocityActualR = shooterTable.getDoubleTopic("VelocityActualRightRPS").publish();
    private static final DoublePublisher shooterVelocityActualL = shooterTable.getDoubleTopic("VelocityActualLeftRPS").publish();

    private static final DoublePublisher shooterSpeedSetpointPub =
        shooterTable.getDoubleTopic("ManualVelocitySetpointRPS").publish();
    private static final DoubleSubscriber shooterSpeedSetpointSub =
        shooterTable.getDoubleTopic("ManualVelocitySetpointRPS").subscribe(Variables.SHOOTER_SPEED);

    // Left-bumper fixed/backup shot setpoint -- deliberately its own topic, not reusing
    // ManualVelocitySetpointRPS above, so dialing in the fixed shot never depends on (or
    // fights with) whatever the manual-override switch happens to be set to.
    private static final DoublePublisher fixedShooterSpeedPub =
        shooterTable.getDoubleTopic("FixedVelocitySetpointRPS").publish();
    private static final DoubleSubscriber fixedShooterSpeedSub =
        shooterTable.getDoubleTopic("FixedVelocitySetpointRPS").subscribe(Variables.FIXED_SHOOTER_SPEED);

    private static final BooleanPublisher manualShooterOverridePub =
        shooterTable.getBooleanTopic("ManualOverride").publish();
    private static final BooleanSubscriber manualShooterOverrideSub =
        shooterTable.getBooleanTopic("ManualOverride").subscribe(false);

    // ---- Feed (upper/lower feed motors, FeedSubsystem) ----
    private static final DoublePublisher feedVelocityUpperRPS = feedTable.getDoubleTopic("VelocityUpperRPS").publish();
    private static final DoublePublisher feedVelocityLowerRPS = feedTable.getDoubleTopic("VelocityLowerRPS").publish();

    // ---- Intake (ground intake rollers, IntakeSubsystem — not the arm) ----
    private static final DoublePublisher intakeVelocityRPS = intakeMotorTable.getDoubleTopic("VelocityRPS").publish();

    // ---- IntakeDrop (the pivoting arm, IntakeDropSubsystem) ----
    // Position: -15 = up, 90 = down (measured from horizontal). Telemetry only. This is an arm
    // angle, not a motor's own rotation, so it stays in degrees rather than RPS.
    private static final DoublePublisher intakeDropPositionDeg =
        intakeTable.getDoubleTopic("PositionDeg").publish();

    // ---- Info / EaseOfLife (not mechanism-specific — match/scoring state) ----
    private static final StringPublisher currentShift = infoTable.getStringTopic("EaseofLife/CurrentShift").publish();
    private static final DoublePublisher shiftTimeRemaining = infoTable.getDoubleTopic("EaseofLife/ShiftTimeRemaining").publish();
    private static final BooleanPublisher isHubActivePub = infoTable.getBooleanTopic("EaseofLife/IsHubActive").publish();
    private static final DoublePublisher distToHubPublisher = infoTable.getDoubleTopic("EaseofLife/DistanceToHub").publish();
    // Whether auto-align-while-shooting (AlignWhileShooting, toggled by the Y button) is on.
    private static final BooleanPublisher alignModePub = infoTable.getBooleanTopic("AlignMode").publish();

    static {
        // Seed defaults so both entries show up on the dashboard immediately,
        // rather than only appearing after the robot happens to publish once.
        shooterSpeedSetpointPub.set(Variables.SHOOTER_SPEED);
        fixedShooterSpeedPub.set(Variables.FIXED_SHOOTER_SPEED);
        manualShooterOverridePub.set(false);
        alignModePub.set(true); // matches EaseofLife's autoAlignEnabled default
        fuelExitSpeedPub.set(Variables.FUEL_EXIT_SPEED_MPS);
    }
    // ---- Robot ----
    public static void putRobotState(String state) { robotState.set(state); }

    // ---- AutoAlign ----
    /**
     * Puts the target angle for AlignToHub
     * @param deg (Degrees)
     */
    public static void putTargetAngle(double deg) { targetAngle.set(deg); }

    public static void putAlignP(double P) { AligntoHubP.set(P); }
    public static void putAlignI(double I) { AligntoHubI.set(I); }
    public static void putAlignD(double D) { AligntoHubD.set(D); }
    public static double getAlignP() { return alignPSub.get(); }
    public static double getAlignI() { return alignISub.get(); }
    public static double getAlignD() { return alignDSub.get(); }

    public static void putAlignWallP(double P) { AligntoWallP.set(P); }
    public static void putAlignWallI(double I) { AligntoWallI.set(I); }
    public static void putAlignWallD(double D) { AligntoWallD.set(D); }
    public static double getAlignWallP() { return alignWallPSub.get(); }
    public static double getAlignWallI() { return alignWallISub.get(); }
    public static double getAlignWallD() { return alignWallDSub.get(); }

    public static void putFuelExitSpeedMps(double mps) { fuelExitSpeedPub.set(mps); }
    public static double getFuelExitSpeedMps() { return fuelExitSpeedSub.get(); }

    // ---- Shooter ----
    public static void putTargetShooterSpeed(double rps) { shooterVelocitySetpoint.set(rps); }
    public static void putShooterVelocityRight(double rps) { shooterVelocityActualR.set(rps); }
    public static void putShooterVelocityLeft(double rps) { shooterVelocityActualL.set(rps); }
    /** Manually-entered shooter speed setpoint from the dashboard (used when manual override is on). */
    public static double getManualShooterSpeed() { return shooterSpeedSetpointSub.get(); }
    public static boolean isManualShooterOverride() { return manualShooterOverrideSub.get(); }
    /** Live-tunable target for the left-bumper fixed/backup shot, see ShooterSubsystem.startFixed(). */
    public static double getFixedShooterSpeed() { return fixedShooterSpeedSub.get(); }

    // ---- Feed ----
    public static void putFeedVelocityUpper(double rps) { feedVelocityUpperRPS.set(rps); }
    public static void putFeedVelocityLower(double rps) { feedVelocityLowerRPS.set(rps); }
    // ---- Intake (rollers) ----
    public static void putIntakeVelocity(double rps) { intakeVelocityRPS.set(rps); }

    // ---- IntakeDrop (arm) ----
    public static void putIntakeDropPositionDegrees(double degrees) { intakeDropPositionDeg.set(degrees); }

    // ---- Info / EaseOfLife ----
    public static void putCurrentShift(String shift) { currentShift.set(shift); }
    public static void putShiftTime(double time) { shiftTimeRemaining.set(time); }
    public static void putIsHubActive(boolean active) { isHubActivePub.set(active); }
    public static void putDisttoHub(double dist) { distToHubPublisher.set(dist); }
    /** Whether auto-align-while-shooting is currently toggled on. */
    public static void putAlignMode(boolean enabled) { alignModePub.set(enabled); }
}