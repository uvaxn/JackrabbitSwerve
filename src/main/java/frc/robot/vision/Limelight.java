package frc.robot.vision;

import static frc.robot.constants.LimelightConstants.AGREED_TRANSLATION_EPSILON_M;
import static frc.robot.constants.LimelightConstants.DEFAULT_STABLE_UPDATE_THRESHOLD;
import static frc.robot.constants.LimelightConstants.HUB_ALIGN_POS_STD_DEV_M;
import static frc.robot.constants.LimelightConstants.HUB_ALIGN_ROTATION_STD_DEV;
import static frc.robot.constants.LimelightConstants.MAX_ESTIMATE_AGE_SECONDS;

import java.util.Optional;

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
 * and toggling hub precision mode. Every tunable number lives in LimelightConstants.java.
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
        // intrinsic (distance/tag-count based) std dev entirely in favor of a flat, simpler
        // number (see LimelightConstants.HUB_ALIGN_*).
        final Matrix<N3, N1> standardDeviations = hubPrecisionMode
                ? VecBuilder.fill(HUB_ALIGN_POS_STD_DEV_M, HUB_ALIGN_POS_STD_DEV_M, HUB_ALIGN_ROTATION_STD_DEV)
                : VecBuilder.fill(fusion.posStdDevBase, fusion.posStdDevBase, fusion.rotationStdDevRad);
        posePublisher.set(fusion.fusedPose);

        latestEstimate = Optional.of(fusion.fusedPose);
        latestEstimateTimestamp = fusion.timestampSeconds;

        return Optional.of(new Measurement(fusion.fusedPose, fusion.timestampSeconds, standardDeviations));
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
}