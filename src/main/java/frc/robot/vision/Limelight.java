package frc.robot.vision;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.vision.LimelightHelpers.PoseEstimate;

public class Limelight extends SubsystemBase {
    private static final AprilTagFieldLayout Afield =
            AprilTagFieldLayout.loadField(
                AprilTagFields.k2026RebuiltAndymark
            );
    
        // ---- field + rejection tunables ----
    private static final double FIELD_LENGTH_M = Afield.getFieldLength();
    private static final double FIELD_WIDTH_M = Afield.getFieldWidth();

    private static final double MAX_SINGLE_TAG_AMBIGUITY = 0.32;
    private static final double MAX_ANGULAR_VEL_DEG_PER_SEC = 540.0;
    private static final double MAX_ACCEPT_DIST_M = 5.0;

    // ---- position (x/y) stddev model, scaled by MT2 distance/tag count ----
    private static final double POS_STD_DEV_FLOOR = 0.1;
    private static final double POS_STD_DEV_DIST_COEFF = 0.08;
    private static final double MULTI_TAG_STD_DEV_SCALE = 0.5;
    private static final double MAX_ACCEPTED_POS_STD_DEV = 3.0;

    // ---- rotation stddev for the MT1-sourced drift correction ----
    // MT2's own rotation just echoes the gyro heading it was fed it is NOT an independent
    // measurement so MT1 (which solves rotation from tag geometry alone) is the only thing
    // here that can actually correct gyro drift. It only gets to do so when trustworthy.
    private static final double MT1_ROTATION_STD_DEV_MULTI_TAG_RAD = 0.3;
    private static final double MT1_ROTATION_STD_DEV_SINGLE_TAG_RAD = 1.0;
    private static final double UNTRUSTED_ROTATION_STD_DEV = 9999999;

    // ---- staleness: how long a fused pose stays valid with nothing refreshing it ----
    private static final double MAX_ESTIMATE_AGE_SECONDS = 0.25;

    private final String name;
    private final NetworkTable telemetryTable;
    private final StructPublisher<Pose2d> posePublisher;

    private Optional<Pose2d> latestEstimate = Optional.empty();
    private double latestEstimateTimestamp = 0.0;
    
    private Optional<Pose2d> latestCameraOnlyPose = Optional.empty();
    private double latestCameraOnlyTimestamp = 0.0;
    private double latestCameraHubDist = 0.0;
    public Limelight(String name) {
        this.name = name;
        this.telemetryTable = NetworkTableInstance.getDefault().getTable(name);
        this.posePublisher = telemetryTable.getStructTopic("EstimatedPose", Pose2d.struct).publish();
    }

    private static boolean isInField(Translation2d t) {
        return t.getX() >= 0 && t.getX() <= FIELD_LENGTH_M &&
               t.getY() >= 0 && t.getY() <= FIELD_WIDTH_M;
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
            DriverStation.reportWarning(
                "out of bounds!", false);
            return Optional.empty();
        }

        if (mt2.avgTagDist > MAX_ACCEPT_DIST_M) {
            return Optional.empty();
        }

        if (Math.toDegrees(Math.abs(angularVelocityRadPerSec)) > MAX_ANGULAR_VEL_DEG_PER_SEC) {
            return Optional.empty();
        }
        // --- MT1 only gets to correct rotation when it's actually trustworthy this cycle ---

        final boolean mt1RotationTrustworthy = mt1 != null &&
            (mt1.tagCount > 1
            || (mt1.rawFiducials.length > 0 
            && mt1.rawFiducials[0].ambiguity <= MAX_SINGLE_TAG_AMBIGUITY));

        final Pose2d fusedPose;
        final double rotationStdDevRad;
        if (mt1RotationTrustworthy) {
            fusedPose = new Pose2d(mt2Translation, mt1.pose.getRotation());
            rotationStdDevRad = (mt1.tagCount > 1)
                ? MT1_ROTATION_STD_DEV_MULTI_TAG_RAD
                : MT1_ROTATION_STD_DEV_SINGLE_TAG_RAD;
                    final Translation2d mt1Translation = mt1.pose.getTranslation();
        if (isInField(mt1Translation) && mt1.avgTagDist <= MAX_ACCEPT_DIST_M) {
            latestCameraOnlyPose =
            Optional.of(
                new Pose2d(
                    mt2.pose.getTranslation(),
                    mt1.pose.getRotation()
                )
            );
        latestCameraOnlyTimestamp = Timer.getFPGATimestamp();
        Translation2d robotPosition = mt1.pose.getTranslation();

        Translation2d hubPosition = Constants.getTeamHubTranslation();

        latestCameraHubDist =
            robotPosition.getDistance(hubPosition);
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

        final Matrix<N3, N1> standardDeviations = VecBuilder.fill(posStdDev, posStdDev, rotationStdDevRad);
        posePublisher.set(fusedPose);

        latestEstimate = Optional.of(fusedPose);
        latestEstimateTimestamp = mt2.timestampSeconds;

        return Optional.of(
            new Measurement(
                fusedPose,
                mt2.timestampSeconds,
                standardDeviations
            )
        );
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

    public static class Measurement {
        public final Pose2d pose;
        public final double timestamp;
        public final Matrix<N3,N1> standardDeviations;

        public Measurement(
            Pose2d pose,
            double timestamp,
            Matrix<N3,N1> standardDeviations) {

            this.pose = pose;
            this.timestamp = timestamp;
            this.standardDeviations = standardDeviations;
        }
        
    }
}