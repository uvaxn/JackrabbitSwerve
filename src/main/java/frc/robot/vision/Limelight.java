package frc.robot.vision;

import static frc.robot.constants.LimelightConstants.AGREED_TRANSLATION_EPSILON_M;
import static frc.robot.constants.LimelightConstants.AUTONOMOUS_RESEED_HEADING_TOLERANCE_DEG;
import static frc.robot.constants.LimelightConstants.DEFAULT_STABLE_UPDATE_THRESHOLD;
import static frc.robot.constants.LimelightConstants.HUB_ALIGN_POS_STD_DEV_M;
import static frc.robot.constants.LimelightConstants.MAX_ESTIMATE_AGE_SECONDS;
import static frc.robot.constants.LimelightConstants.TRUST_CURVE_EXPONENT;
import static frc.robot.constants.LimelightConstants.TRUST_PERCENT_RESEED;
import static frc.robot.constants.LimelightConstants.VISION_ROTATION_STD_DEV;
import static frc.robot.constants.LimelightConstants.VISION_TRUST_DISABLED_STD_DEV;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.vision.LimelightHelpers.PoseEstimate;
import frc.robot.vision.PoseFusion.FusionResult;

/**
 * The Limelight subsystem. Talks to LimelightHelpers, hands the raw MT1/MT2 estimates to
 * PoseFusion for the actual fusion/rejection math (see that file), and owns everything
 * stateful: caching the latest accepted pose, tracking pose stability, publishing telemetry,
 * toggling hub precision mode, and converting a 0-100 vision trust percentage into the
 * position std dev actually handed to the pose estimator. Every tunable number lives in
 * LimelightConstants.java.
 */
public class Limelight extends SubsystemBase {

    private final String name;
    private final NetworkTable telemetryTable;
    private final StructPublisher<Pose2d> posePublisher;
    private final IntegerPublisher stableUpdatesPublisher;

    private Optional<Pose2d> latestEstimate = Optional.empty();
    private double latestEstimateTimestamp = 0.0;

    private Optional<Pose2d> latestCameraOnlyPose = Optional.empty();
    private double latestCameraOnlyTimestamp = 0.0;
    private double latestCameraHubDist = 0.0;

    private int numStableUpdates = 0;
    private boolean hubPrecisionMode = false;

    // The knob: 0-100, see setVisionTrustPercent(). 50 is a deliberate no-op default -- it's
    // the "Normal" operating mode percentage, and maps to an exact 1x multiplier below, so a
    // freshly-constructed Limelight behaves identically to before this system existed until
    // something calls setVisionTrustPercent().
    private double visionTrustPercent = 50.0;

    public Limelight(String name) {
        this.name = name;
        this.telemetryTable = NetworkTableInstance.getDefault().getTable(name);
        this.posePublisher = telemetryTable.getStructTopic("EstimatedPose", Pose2d.struct).publish();
        this.stableUpdatesPublisher = telemetryTable.getIntegerTopic("NumStableUpdates").publish();
    }

    /**
     * @param currentRobotPose         current pose estimate; its rotation seeds MegaTag2's yaw
     * @param angularVelocityRadPerSec current robot angular velocity in rad/s, used to reject
     *                                 updates while spinning too fast for MegaTag2 to trust
     */
    public Optional<Measurement> getMeasurement(Pose2d currentRobotPose, double angularVelocityRadPerSec) {
        LimelightHelpers.SetRobotOrientation(name, currentRobotPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

        final PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        final PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

        final Optional<FusionResult> result = PoseFusion.fuse(mt1, mt2, angularVelocityRadPerSec);
        if (result.isEmpty()) {
            return Optional.empty();
        }
        final FusionResult fusion = result.get();

        // --- pose stability: has this cycle's fused pose landed near where we already
        // thought we were? Consecutive agreement builds confidence, any miss resets it. A
        // rejected/missing cycle (the isEmpty() check above) neither adds to nor resets this.
        if (fusion.fusedPose.getTranslation().getDistance(currentRobotPose.getTranslation())
                < AGREED_TRANSLATION_EPSILON_M) {
            numStableUpdates++;
        } else {
            numStableUpdates = 0;
        }
        stableUpdatesPublisher.set(numStableUpdates);

        if (fusion.cameraOnlyPose.isPresent() && fusion.mt1TranslationForHubDistance.isPresent()) {
            latestCameraOnlyPose = fusion.cameraOnlyPose;
            latestCameraOnlyTimestamp = Timer.getFPGATimestamp();
            latestCameraHubDist = fusion.mt1TranslationForHubDistance.get()
                    .getDistance(Constants.getTeamHubTranslation());
        }

        // hub precision mode is subsystem state, not fusion math, so it's applied here on top
        // of PoseFusion's result rather than inside PoseFusion: while it's on, skip the
        // trust-percentage system below entirely in favor of one flat, simpler number (see
        // LimelightConstants.HUB_ALIGN_POS_STD_DEV_M). Otherwise, scale PoseFusion's adaptive
        // fusion.posStdDevBase by the current trust percentage -- see
        // calculateVisionPositionStdDev() for the curve.
        final double posStdDev = hubPrecisionMode
                ? HUB_ALIGN_POS_STD_DEV_M
                : calculateVisionPositionStdDev(fusion.posStdDevBase);

        // Rotation is never corrected by vision through this system, regardless of trust
        // percentage or hub precision mode -- see VISION_ROTATION_STD_DEV's doc. PoseFusion
        // still computes fusion.rotationStdDevRad above (MT1/MT2 fusion stays fully intact),
        // it's just not surfaced into the measurement handed back below anymore.
        final Matrix<N3, N1> standardDeviations =
                VecBuilder.fill(posStdDev, posStdDev, VISION_ROTATION_STD_DEV);
        posePublisher.set(fusion.fusedPose);

        latestEstimate = Optional.of(fusion.fusedPose);
        latestEstimateTimestamp = fusion.timestampSeconds;

        return Optional.of(new Measurement(fusion.fusedPose, fusion.timestampSeconds, standardDeviations));
    }

    /**
     * Maps the current visionTrustPercent (0-100) onto a position std dev, by scaling
     * posStdDevBase -- PoseFusion's adaptive, distance/tag-count-aware estimate -- with a
     * logit/odds-ratio curve: {@code multiplier = ((1 - trust) / trust) ^ TRUST_CURVE_EXPONENT}
     * where trust = visionTrustPercent / 100. That shape is what makes 50% land on exactly 1x
     * and 25% land on exactly 2x (see TRUST_CURVE_EXPONENT's derivation), while still
     * producing a smooth, monotonic multiplier everywhere else: it falls toward 0 as trust
     * rises toward 100% (vision essentially unopposed) and rises toward infinity as trust
     * falls toward 0% (vision effectively ignored), both real limits of one formula rather
     * than a piecewise special case -- the explicit trust <= 0 branch below exists purely for
     * readability, Math.min already saturates correctly on its own since Java's 1.0/0.0 is a
     * well-defined (and here, harmless) IEEE 754 infinity, not a thrown exception.
     */
    private double calculateVisionPositionStdDev(double posStdDevBase) {
        double trust = MathUtil.clamp(visionTrustPercent, 0.0, 100.0) / 100.0;
        if (trust <= 0.0) {
            return VISION_TRUST_DISABLED_STD_DEV;
        }
        double multiplier = Math.pow((1.0 - trust) / trust, TRUST_CURVE_EXPONENT);
        return Math.min(posStdDevBase * multiplier, VISION_TRUST_DISABLED_STD_DEV);
    }

    public Optional<Pose2d> getCameraOnlyPose() {
        if (latestCameraOnlyPose.isEmpty()) {
            return Optional.empty();
        }
        if (Timer.getFPGATimestamp() - latestCameraOnlyTimestamp > MAX_ESTIMATE_AGE_SECONDS) {
            return Optional.empty();
        }
        return latestCameraOnlyPose;
    }

    public double getDistanceToHub() {
        if (latestCameraOnlyPose.isEmpty()) {
            return 3.0; // default fallback
        }
        if (Timer.getFPGATimestamp() - latestCameraOnlyTimestamp > MAX_ESTIMATE_AGE_SECONDS) {
            return 3.0;
        }
        // MegaTag1 is used for hub distance (instead of MegaTag2) because though MegaTag2
        // gives significantly better pose reads, MegaTag1 is substantially better for hub
        // distance specifically: despite its own pose ambiguity, it holds distance-from-hub
        // steady, and MegaTag2's distance can be thrown off if the robot's yaw estimate is off.
        return latestCameraHubDist;
    }

    public Optional<Pose2d> getEstimatedPose() {
        if (latestEstimate.isEmpty()) {
            return Optional.empty();
        }
        if (Timer.getFPGATimestamp() - latestEstimateTimestamp > MAX_ESTIMATE_AGE_SECONDS) {
            return Optional.empty();
        }
        return latestEstimate;
    }

    /** @return how many consecutive accepted vision updates have landed within
     *  AGREED_TRANSLATION_EPSILON_M of the pose we already had, resets to 0 on any miss.
     *  A rejected/missing cycle neither adds to nor resets this, it just holds. */
    public int getNumStableUpdates() {
        return numStableUpdates;
    }

    /** @return true once the default number of consecutive updates have agreed. */
    public boolean isPoseStable() {
        return numStableUpdates >= DEFAULT_STABLE_UPDATE_THRESHOLD;
    }

    /** @param updates a custom threshold, for callers that want to demand more or less
     *  agreement than the default before trusting this camera. */
    public boolean isPoseStable(int updates) {
        return numStableUpdates >= updates;
    }

    /**
     * Switches position std dev to the flat HUB_ALIGN_POS_STD_DEV_M value (see
     * LimelightConstants) instead of PoseFusion's adaptive distance/tag-count model, while a
     * HUB-targeting alignment command is actively running. Call with true whenever a cycle is
     * aiming at the HUB (e.g. AlignToHub.initialize(), or AlignWhileShooting while it's in its
     * HUB-facing branch) and false otherwise (that command's end(), or AlignWhileShooting's
     * wall-facing branch). Rotation trust is untouched, this only affects position.
     * <p>
     * Not to be confused with the driver-facing "AlignMode" NetworkTables toggle published by
     * EaseofLife -- that one is "should we auto-aim at all", this is "how much do we trust
     * vision position while we do."
     */
    public void setHubPrecisionMode(boolean aligning) {
        this.hubPrecisionMode = aligning;
    }

    public boolean isHubPrecisionMode() {
        return hubPrecisionMode;
    }

    /**
     * Sets how much the next vision measurements should be trusted, 0 (ignore vision) to 100
     * (trust it essentially unopposed) -- see calculateVisionPositionStdDev() for the curve
     * this feeds, and LimelightConstants.TRUST_PERCENT_* for the standard operating-mode
     * values. Out-of-range input is clamped rather than left to produce odd behavior.
     * Doesn't affect hub precision mode, that's a separate override -- see setHubPrecisionMode.
     */
    public void setVisionTrustPercent(double percent) {
        this.visionTrustPercent = MathUtil.clamp(percent, 0.0, 100.0);
    }

    public double getVisionTrustPercent() {
        return visionTrustPercent;
    }

    /**
     * Autonomous-alignment reseed check. An alignment command's execute() should call this
     * every cycle while it's running in autonomous; every other cycle this returns
     * Optional.empty() and does nothing. The one cycle both alreadyReseeded is still false AND
     * the robot is within AUTONOMOUS_RESEED_HEADING_TOLERANCE_DEG of its target heading AND
     * isPoseStable(), this performs a single hard pose reset (not a soft addVisionMeasurement
     * nudge) from the current vision estimate: it briefly raises visionTrustPercent to
     * LimelightConstants.TRUST_PERCENT_RESEED for that one moment, hands back the pose to
     * reset to, and immediately restores visionTrustPercent to normalOperatingTrustPercent --
     * this is a one-shot bracket around a single reset, not a standing trust change.
     * <p>
     * This method does NOT track "have I already reseeded" itself -- alignment commands come
     * and go (a fresh one starts a fresh attempt), so that latch has to live in the caller,
     * reset in that command's own initialize(). Pass it back in every cycle; once this returns
     * a present Optional, set your local latch to true and stop calling (or keep calling, it's
     * a no-op once alreadyReseeded is true either way).
     *
     * @param headingErrorDegrees      |current heading - target heading|, from the caller's own math
     * @param alreadyReseeded          the caller's own one-shot latch
     * @param normalOperatingTrustPercent what to restore visionTrustPercent to right after
     *                                 (typically whatever LimelightConstants.TRUST_PERCENT_*
     *                                 matches the current operating mode)
     * @return the pose to hard-reset the drivetrain to, only on the one cycle a reseed fires
     */
    public Optional<Pose2d> checkAutonomousReseed(
            double headingErrorDegrees, boolean alreadyReseeded, double normalOperatingTrustPercent) {

        if (alreadyReseeded
                || headingErrorDegrees > AUTONOMOUS_RESEED_HEADING_TOLERANCE_DEG
                || !isPoseStable()) {
            return Optional.empty();
        }

        // getEstimatedPose(), not the raw latestEstimate field: isPoseStable() alone doesn't
        // guarantee freshness on its own -- a gap of rejected/missing cycles holds the stable
        // count rather than resetting it (see its doc above), so it's still possible to be
        // "stable" while latestEstimate has quietly gone stale. Too consequential a reset to
        // skip that check.
        Optional<Pose2d> reseedPose = getEstimatedPose();
        if (reseedPose.isEmpty()) {
            return Optional.empty();
        }

        setVisionTrustPercent(TRUST_PERCENT_RESEED);
        setVisionTrustPercent(normalOperatingTrustPercent);
        return reseedPose;
    }
}