// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import org.photonvision.targeting.PhotonPipelineResult;

public class CameraSubsystem extends SubsystemBase {
  /** Creates a new PhotonVision. */

  private CommandSwerveDrivetrain swerveDrive;
  public StructPublisher<Pose3d> EstimatedPosition;
  public final PhotonCamera camera;
  private final PhotonPoseEstimator photonEstimator;
    private final Field2d field = new Field2d();
  private final AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
    
    public CameraSubsystem(CommandSwerveDrivetrain swerveDrive) {
        this.swerveDrive = swerveDrive;
        camera = new PhotonCamera("Limelight4");
        SmartDashboard.putData("Field", field);
        photonEstimator = new PhotonPoseEstimator(aprilTagLayout, PoseStrategy.LOWEST_AMBIGUITY, Constants.kcamToRobot);
        EstimatedPosition = NetworkTableInstance.getDefault().getStructTopic("EstimatedPose", Pose3d.struct).publish();
        // move tag poses here
        field.getObject("tags").setPoses(
            aprilTagLayout.getTags().stream()
                .map(t -> t.pose.toPose2d())
                .collect(java.util.stream.Collectors.toList())
        );
        Thread visionThread = new Thread(() -> {
            while (!Thread.interrupted()) {
                cachedResults = camera.getAllUnreadResults();
                try { Thread.sleep(20); } catch (InterruptedException e) { break; }
            }
        });
        visionThread.setDaemon(true);
        visionThread.start();
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
    @Override
    public void periodic() {
        for (PhotonPipelineResult result : cachedResults) {
            photonEstimator.update(result).ifPresent(est -> {
                Pose2d estimatedPose2d = est.estimatedPose.toPose2d();
                if (swerveDrive.getState().Pose.getTranslation()
                        .getDistance(estimatedPose2d.getTranslation()) > 1.0) return;
                swerveDrive.addVisionMeasurement(
                    estimatedPose2d,
                    est.timestampSeconds,
                    VecBuilder.fill(0.3, 0.3, 9999999)
                );
                EstimatedPosition.set(est.estimatedPose);
                field.setRobotPose(estimatedPose2d);
            });
        }
    }
}
