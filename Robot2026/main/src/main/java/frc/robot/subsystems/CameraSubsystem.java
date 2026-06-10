package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.EstimatedRobotPose;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Vars;

public class CameraSubsystem extends SubsystemBase {
    private final CommandSwerveDrivetrain swerveDrive;

    private final PhotonCamera camera = new PhotonCamera("Limelight");
    private final Transform3d robotToCamera = Vars.kcamToRobot;
    private final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
    private final PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(fieldLayout, robotToCamera);

    private Optional<EstimatedRobotPose> latestEstimate = Optional.empty();
    private PhotonPipelineResult latestResult = null;

    public CameraSubsystem(CommandSwerveDrivetrain swerveDrive) {
        this.swerveDrive = swerveDrive;
    }

    @Override
    public void periodic() {
        var results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            latestResult = results.get(results.size() - 1);
            latestEstimate = poseEstimator.estimateLowestAmbiguityPose(latestResult);

            latestEstimate.ifPresent(est ->
                swerveDrive.addVisionMeasurement(
                    est.estimatedPose.toPose2d(),
                    est.timestampSeconds
                )
            );
        }
    }

    public Optional<EstimatedRobotPose> getEstimatedPose() {
        return latestEstimate;
    }

    public Optional<Pose2d> getNearestTagPose(Pose2d robotPose) {
        if (latestResult == null || !latestResult.hasTargets()) return Optional.empty();

        return latestResult.getTargets().stream()
            .map(target -> fieldLayout.getTagPose(target.getFiducialId()))
            .filter(Optional::isPresent)
            .map(Optional::get)
            .map(tagPose3d -> tagPose3d.toPose2d())
            .min((a, b) -> {
                double distA = a.getTranslation().getDistance(robotPose.getTranslation());
                double distB = b.getTranslation().getDistance(robotPose.getTranslation());
                return Double.compare(distA, distB);
            });
    }
}