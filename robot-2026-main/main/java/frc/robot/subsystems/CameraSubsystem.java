// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import java.util.ArrayList;
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
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.photonvision.targeting.PhotonPipelineResult;

public class CameraSubsystem extends SubsystemBase {
    private StructPublisher<Pose3d> EstimatedPosition;
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator;
    private static final AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    private List<EstimatedRobotPose> latestEstimates = List.of();
    private List<PhotonPipelineResult> cachedResults = List.of();
    private final StructArrayPublisher<Pose2d> visibleTagPublisher =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("VisibleTags", Pose2d.struct)
            .publish();
    public CameraSubsystem() {
        camera = new PhotonCamera("limelight4");
        photonEstimator = new PhotonPoseEstimator(aprilTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.kcamToRobot);

        photonEstimator.setMultiTagFallbackStrategy( // if multitag doesnt work for ANY reason
            PoseStrategy.LOWEST_AMBIGUITY
        );
        EstimatedPosition = NetworkTableInstance.getDefault().getStructTopic("EstimatedPose", Pose3d.struct).publish();
        
    }

    public Optional<Pose2d> getTagPose2d(int id) {
        var tag = aprilTagLayout.getTagPose(id);
        if (tag.isPresent()) {
            return Optional.of(tag.get().toPose2d());
        }
        
        return Optional.empty();
    }
    
    public List<PhotonPipelineResult> getCachedResults() {
        return cachedResults;
    }
    public List<EstimatedRobotPose> getEstimatedGlobalPoses() {
        return latestEstimates;
    }
    

    @Override
  public void periodic() {
    if (edu.wpi.first.wpilibj.RobotBase.isSimulation()) return;
      var results = camera.getAllUnreadResults();
      cachedResults = results;
      List<EstimatedRobotPose> frameEstimates = new ArrayList<>();
      for (PhotonPipelineResult result : results) {
          Optional<EstimatedRobotPose> visionEstimate = photonEstimator.update(result);
          if (visionEstimate.isPresent()) {
              Pose2d pose = visionEstimate.get().estimatedPose.toPose2d();
              if (pose.getX() < 0 || pose.getX() > 17.5 ||
                  pose.getY() < 0 || pose.getY() > 8.0) continue;
              frameEstimates.add(visionEstimate.get());
              EstimatedPosition.set(              
                  visionEstimate.get().estimatedPose,
                  (long)(visionEstimate.get().timestampSeconds * 1e6)
              );
          }
      }
      latestEstimates = frameEstimates;
      List<Pose2d> visiblePoses = new ArrayList<>();
    List<Double> visibleTagIds = new ArrayList<>();
            for (PhotonPipelineResult result : cachedResults) {
                if (!result.hasTargets()) continue;

                for (var target : result.getTargets()) {
                    int id = target.getFiducialId();

                    visibleTagIds.add((double) id);

                    getTagPose2d(id).ifPresent(visiblePoses::add);
                }
            }
            visibleTagPublisher.set(visiblePoses.toArray(new Pose2d[0]));
            SmartDashboard.putNumberArray("Vision/TagsVisible", //bascilly outputs which tags it's currently seeing
            visibleTagIds.stream().mapToDouble(Double::doubleValue).toArray());

  }
}