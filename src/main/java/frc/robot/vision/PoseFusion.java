package frc.robot.vision;

import static frc.robot.constants.LimelightConstants.MAX_ACCEPT_DIST_M;
import static frc.robot.constants.LimelightConstants.MAX_ANGULAR_VEL_DEG_PER_SEC;
import static frc.robot.constants.LimelightConstants.MAX_SINGLE_TAG_AMBIGUITY;
import static frc.robot.constants.LimelightConstants.MULTI_TAG_STD_DEV_SCALE;
import static frc.robot.constants.LimelightConstants.MT1_ROTATION_STD_DEV_MULTI_TAG_RAD;
import static frc.robot.constants.LimelightConstants.MT1_ROTATION_STD_DEV_SINGLE_TAG_RAD;
import static frc.robot.constants.LimelightConstants.POS_STD_DEV_DIST_COEFF;
import static frc.robot.constants.LimelightConstants.POS_STD_DEV_FLOOR;
import static frc.robot.constants.LimelightConstants.MAX_ACCEPTED_POS_STD_DEV;
import static frc.robot.constants.LimelightConstants.UNTRUSTED_ROTATION_STD_DEV;
import static frc.robot.constants.LimelightConstants.FIELD_LENGTH_M;
import static frc.robot.constants.LimelightConstants.FIELD_WIDTH_M;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.vision.LimelightHelpers.PoseEstimate;
import java.util.Optional;

/**
 * The actual MegaTag1/MegaTag2 fusion and rejection math, pulled out of Limelight.java so it
 * reads (and could be tested) on its own with no NetworkTables or subsystem state involved.
 * MT2 gates whether this cycle's position is trusted at all, MT1 only gets to correct
 * rotation when it is independently trustworthy this cycle. See LimelightConstants.java for
 * every threshold referenced here, and Limelight.java for what happens to the result.
 */
public final class PoseFusion {
    private PoseFusion() {}

    private static boolean isInField(Translation2d t) {
        return t.getX() >= 0 && t.getX() <= FIELD_LENGTH_M
                && t.getY() >= 0 && t.getY() <= FIELD_WIDTH_M;
    }

    /**
     * @param mt1 MegaTag1 estimate (rotation solved from tag geometry alone), may be null
     * @param mt2 MegaTag2 estimate (rotation just echoes the gyro heading it was seeded with)
     * @param angularVelocityRadPerSec current robot angular velocity, used to reject updates
     *                                 while spinning too fast for MegaTag2 to trust
     */
    public static Optional<FusionResult> fuse(
            PoseEstimate mt1, PoseEstimate mt2, double angularVelocityRadPerSec) {

        if (mt2 == null || mt2.tagCount == 0) {
            return Optional.empty();
        }

        // --- MT2 gates whether we trust this cycle's position at all ---

        if (mt2.tagCount == 1 && mt2.rawFiducials.length > 0
                && mt2.rawFiducials[0].ambiguity > MAX_SINGLE_TAG_AMBIGUITY) {
            return Optional.empty();
        }

        final Translation2d mt2Translation = mt2.pose.getTranslation();
        if (!isInField(mt2Translation)) {
            DriverStation.reportWarning("out of bounds!", false);
            return Optional.empty();
        }

        if (mt2.avgTagDist > MAX_ACCEPT_DIST_M) {
            return Optional.empty();
        }

        if (Math.toDegrees(Math.abs(angularVelocityRadPerSec)) > MAX_ANGULAR_VEL_DEG_PER_SEC) {
            return Optional.empty();
        }

        // --- MT1 only gets to correct rotation when it's actually trustworthy this cycle ---

        final boolean mt1RotationTrustworthy = mt1 != null
                && (mt1.tagCount > 1
                        || (mt1.rawFiducials.length > 0
                                && mt1.rawFiducials[0].ambiguity <= MAX_SINGLE_TAG_AMBIGUITY));

        final Pose2d fusedPose;
        final double rotationStdDevRad;
        Optional<Pose2d> cameraOnlyPose = Optional.empty();
        Optional<Translation2d> mt1TranslationForHubDistance = Optional.empty();

        if (mt1RotationTrustworthy) {
            final Rotation2d mt1Rotation = mt1.pose.getRotation();
            fusedPose = new Pose2d(mt2Translation, mt1Rotation);
            rotationStdDevRad = (mt1.tagCount > 1)
                    ? MT1_ROTATION_STD_DEV_MULTI_TAG_RAD
                    : MT1_ROTATION_STD_DEV_SINGLE_TAG_RAD;

            // MT1's own translation is separately gated here, it's noisier than MT2's, so it
            // only feeds these two outputs when it clears the same in-field/distance bar on
            // its own merits. cameraOnlyPose intentionally still uses MT2's translation
            // (better) paired with MT1's rotation, mt1TranslationForHubDistance is the raw
            // MT1 translation, kept separate on purpose, see Limelight.getDistanceToHub().
            final Translation2d mt1Translation = mt1.pose.getTranslation();
            if (isInField(mt1Translation) && mt1.avgTagDist <= MAX_ACCEPT_DIST_M) {
                cameraOnlyPose = Optional.of(new Pose2d(mt2Translation, mt1Rotation));
                mt1TranslationForHubDistance = Optional.of(mt1Translation);
            }
        } else {
            // Keep MT2's own (gyro-echoing) rotation and tell the estimator this measurement
            // says nothing about rotation this cycle
            fusedPose = mt2.pose;
            rotationStdDevRad = UNTRUSTED_ROTATION_STD_DEV;
        }

        // --- position stddev, scaled by MT2 distance and tag count ---

        double posStdDev = POS_STD_DEV_FLOOR + POS_STD_DEV_DIST_COEFF * mt2.avgTagDist * mt2.avgTagDist;
        if (mt2.tagCount > 1) {
            posStdDev *= MULTI_TAG_STD_DEV_SCALE;
        }
        posStdDev = Math.min(posStdDev, MAX_ACCEPTED_POS_STD_DEV);

        return Optional.of(new FusionResult(
                fusedPose,
                rotationStdDevRad,
                posStdDev,
                cameraOnlyPose,
                mt1TranslationForHubDistance,
                mt2.timestampSeconds));
    }

    /**
     * Everything Limelight.java needs out of one fusion cycle to update its own state and
     * hand a Measurement back to its caller. posStdDevBase has NOT had align mode applied,
     * that's subsystem state, not fusion math, Limelight.java scales it on top of this.
     */
    public static final class FusionResult {
        public final Pose2d fusedPose;
        public final double rotationStdDevRad;
        public final double posStdDevBase;
        public final Optional<Pose2d> cameraOnlyPose;
        public final Optional<Translation2d> mt1TranslationForHubDistance;
        public final double timestampSeconds;

        public FusionResult(
                Pose2d fusedPose,
                double rotationStdDevRad,
                double posStdDevBase,
                Optional<Pose2d> cameraOnlyPose,
                Optional<Translation2d> mt1TranslationForHubDistance,
                double timestampSeconds) {
            this.fusedPose = fusedPose;
            this.rotationStdDevRad = rotationStdDevRad;
            this.posStdDevBase = posStdDevBase;
            this.cameraOnlyPose = cameraOnlyPose;
            this.mt1TranslationForHubDistance = mt1TranslationForHubDistance;
            this.timestampSeconds = timestampSeconds;
        }
    }
}