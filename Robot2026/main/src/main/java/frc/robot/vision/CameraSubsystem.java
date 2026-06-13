// Note, While photon vision is the OS currently being used, i do plan on switching over to Limelight OS simply because
// of the IMU and elevated hardware capabilities with the Limelight 4 that I am not taking advantage of with photon vision.
// Megatag2 is also pretty good.

package frc.robot.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.EstimatedRobotPose;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Vars;
import frc.robot.controls.EaseofLife;
import frc.robot.subsystems.CommandSwerveDrivetrain;

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
        if (pose.isEmpty()) return 3;

        Translation2d hubTarget = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
            ? Constants.redHubPosition
            : Constants.blueHubPosition;

        Pose2d robotPose = pose.get();
        Translation2d t = robotPose.getTranslation();

        // filter out garbage poses outside field bounds
        if (t.getX() < 0 || t.getX() > 16.54 ||
            t.getY() < 0 || t.getY() > 8.21) {
                DriverStation.reportWarning(
        "getDistanceToHub() OUT OF BOUNDS POSE (x=" + String.format("%.2f", t.getX()) + 
        " y=" + String.format("%.2f", t.getY()) + ")", false);
            return 3;
        }

        return hubTarget.minus(t).getNorm();
    }
    @Override
    
    public void periodic() {
        var results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            latestResult = results.get(results.size() - 1); // because why not
            latestEstimate = poseEstimator.estimateLowestAmbiguityPose(latestResult);
            double dist = getDistanceToHub();
            double stdDev = 0.5 * dist * dist; // scales trust down exponentially with distance
            // TODO: do the thing that enables multi tag in photon settings
            latestEstimate.ifPresent(est ->
                swerveDrive.addVisionMeasurement(
                    est.estimatedPose.toPose2d(),
                    est.timestampSeconds,
                    VecBuilder.fill(stdDev, stdDev, 9999999)
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