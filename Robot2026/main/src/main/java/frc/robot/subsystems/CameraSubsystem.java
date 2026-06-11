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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Vars;

public class CameraSubsystem extends SubsystemBase {
    private final CommandSwerveDrivetrain swerveDrive;

    private final PhotonCamera camera = new PhotonCamera("Limelight");
    private final Transform3d robotToCamera = Vars.kRobotToCam;
    private final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
    // TODO: import the json file for the Andymark field
    private final PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(fieldLayout, robotToCamera);

    private Optional<EstimatedRobotPose> latestEstimate = Optional.empty();
    private PhotonPipelineResult latestResult = null;

    public CameraSubsystem(CommandSwerveDrivetrain swerveDrive) {
        this.swerveDrive = swerveDrive;
    }
    public double getDistanceToHub() {
        Optional<Pose2d> pose = swerveDrive.samplePoseAt(Timer.getFPGATimestamp());
        if (pose.isEmpty() ) return 3; // returns a default distance of 3 meters TODO: add the field's coordinates just so that it doesn't return a distance of 92536 meters or something
        Translation2d hubTarget = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
            ? Constants.redHubPosition
            : Constants.blueHubPosition;
        Pose2d robotPose = pose.get();
        double toHub = hubTarget.minus(robotPose.getTranslation()).getNorm();
        return toHub;
    }
    @Override
    public void periodic() {
        var results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            latestResult = results.get(results.size() - 1); // because why not
            latestEstimate = poseEstimator.estimateLowestAmbiguityPose(latestResult);
            // TODO: do the thing that enables multi tag in photon settings
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