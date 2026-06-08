package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.EstimatedRobotPose;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CameraSubsystem extends SubsystemBase {

    private final PhotonCamera camera = new PhotonCamera("Limelight");

    private final AprilTagFieldLayout fieldLayout =
        AprilTagFields.k2026RebuiltAndymark.loadAprilTagLayoutField();

    private final PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(
        fieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        Constants.kcamToRobot  // pulled from Constants, not hardcoded
    );

    private Optional<EstimatedRobotPose> latestEstimate = Optional.empty();

    @Override
    public void periodic() {
        latestEstimate = poseEstimator.update(camera.getLatestResult());
    }

    /** Returns the latest estimated robot pose from PhotonVision. */
    public Optional<EstimatedRobotPose> getEstimatedPose() {
        return latestEstimate;
    }

    /**
     * Returns the pose of the nearest visible AprilTag to the given robot pose,
     * or empty if no tags are currently detected.
     */
    public Optional<Pose2d> getNearestTagPose(Pose2d robotPose) {
        var result = camera.getLatestResult();
        if (!result.hasTargets()) return Optional.empty();

        return result.getTargets().stream()
            .map(target -> fieldLayout.getTagPose(target.getFiducialId()))
            .filter(Optional::isPresent)
            .map(Optional::get)
            .map(tagPose3d -> tagPose3d.toPose2d())
            .min((a, b) -> Double.compare(
                a.getTranslation().getDistance(robotPose.getTranslation()),
                b.getTranslation().getDistance(robotPose.getTranslation())
            ));
    }
}