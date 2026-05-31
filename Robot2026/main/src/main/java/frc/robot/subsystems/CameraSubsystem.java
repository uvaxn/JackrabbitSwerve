// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import org.photonvision.targeting.PhotonPipelineResult;

public class CameraSubsystem extends SubsystemBase {
  /** Creates a new PhotonVision. */

  private CommandSwerveDrivetrain swerveDrive;
  public StructPublisher<Pose3d> EstimatedPosition;
  public final PhotonCamera camera;
  private final PhotonPoseEstimator photonEstimator;

  private final AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

  public CameraSubsystem(CommandSwerveDrivetrain swerveDrive) {
    this.swerveDrive = swerveDrive;
    camera = new PhotonCamera("Limelight4");
    photonEstimator = new PhotonPoseEstimator(aprilTagLayout, PoseStrategy.LOWEST_AMBIGUITY, Constants.kcamToRobot);
    EstimatedPosition = NetworkTableInstance.getDefault().getStructTopic("EstimatedPose", Pose3d.struct).publish();
  }
    public Optional<Pose2d> getTagPose2d(int id) {
        var tag = aprilTagLayout.getTagPose(id);
        if (tag.isPresent()) {
            return Optional.of(tag.get().toPose2d());
        }
        return Optional.empty();
    }

  private List<PhotonPipelineResult> cachedResults = List.of();
  public List<PhotonPipelineResult> getCachedResults() {
    return cachedResults;
}


public Optional<Pose2d> getNearestTagPose(Pose2d robotPose) {
    double bestDistance = Double.MAX_VALUE;
    Optional<Pose2d> nearest = Optional.empty();

    for (PhotonPipelineResult result : cachedResults) {
        if (!result.hasTargets()) continue;

        for (var target : result.getTargets()) {
            int id = target.getFiducialId();
            if (id <= 0) continue;

            Optional<Pose2d> tagPose = getTagPose2d(id);
            if (tagPose.isEmpty()) continue;

            double dist = robotPose.getTranslation()
                                   .getDistance(tagPose.get().getTranslation());
            if (dist < bestDistance) {
                bestDistance = dist;
                nearest = tagPose;
            }
        }
    }
    return nearest;
}


    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> visionEstimate = Optional.empty();
        for (PhotonPipelineResult result : cachedResults) {
            visionEstimate = photonEstimator.update(result);
        }
        return visionEstimate;
    }


    @Override
    public void periodic() {
        cachedResults = camera.getAllUnreadResults(); 

        Optional<EstimatedRobotPose> estimatedPose = getEstimatedGlobalPose();
        if (estimatedPose.isPresent()) {
            swerveDrive.addVisionMeasurement(
                estimatedPose.get().estimatedPose.toPose2d(),
                estimatedPose.get().timestampSeconds
            );
            EstimatedPosition.set(estimatedPose.get().estimatedPose);
        }
    }
}
