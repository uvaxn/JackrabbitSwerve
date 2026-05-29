// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import frc.robot.Constants;

public class PositionSubsystem extends SubsystemBase {

    private final CommandSwerveDrivetrain swerveDrive;
    private final CameraSubsystem camera;

    private final SwerveDriveOdometry odometry;

    // Visible tag positions from odometry projection
    private final StructArrayPublisher<Pose2d> odometryTagPublisher =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("OdometryTags", Pose2d.struct)
            .publish();

    // Raw vision pose — renamed to avoid conflict with CameraSubsystem's "EstimatedPose"
    private final StructPublisher<Pose3d> rawVisionPosePublisher =
        NetworkTableInstance.getDefault()
            .getStructTopic("Vision/RawPose", Pose3d.struct)
            .publish();

    // Fused odometry + vision pose
    private final StructPublisher<Pose3d> fusedPosePublisher =
        NetworkTableInstance.getDefault()
            .getStructTopic("FusedPose", Pose3d.struct)
            .publish();

    // Pure encoder/gyro odometry pose
    private final StructPublisher<Pose3d> odometryPosePublisher =
        NetworkTableInstance.getDefault()
            .getStructTopic("OdometryPose", Pose3d.struct)
            .publish();

    public PositionSubsystem(CommandSwerveDrivetrain swerveDrive, CameraSubsystem camera) {
        this.swerveDrive = swerveDrive;
        this.camera = camera;

        odometry = new SwerveDriveOdometry(
            swerveDrive.getKinematics(),
            swerveDrive.getState().RawHeading,
            swerveDrive.getState().ModulePositions
        );
    }

    private Matrix<N3, N1> getStdDevs(EstimatedRobotPose estimate) {
        var targets = estimate.targetsUsed;
        int tagCount = targets.size();
        double avgDist = targets.stream()
            .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
            .average()
            .orElse(0);

        if (tagCount >= 2) {
            return VecBuilder.fill(0.2, 0.2, 0.05);
        } else if (tagCount == 1 && avgDist <= 4) {
            return VecBuilder.fill(0.7, 0.7, 9999);
        } else {
            return VecBuilder.fill(4.0, 4.0, 9999);
        }
    }

    @Override
    public void periodic() {

        // Update and publish pure odometry
        odometry.update(
            swerveDrive.getState().RawHeading,
            swerveDrive.getState().ModulePositions
        );
        odometryPosePublisher.set(new Pose3d(odometry.getPoseMeters()));

        // Feed vision into CTRE fused estimator
        for (EstimatedRobotPose estimate : camera.getEstimatedGlobalPoses()) {
            rawVisionPosePublisher.set(estimate.estimatedPose);
            swerveDrive.addVisionMeasurement(
                estimate.estimatedPose.toPose2d(),
                estimate.timestampSeconds,
                getStdDevs(estimate)
            );
        }

        // Publish fused pose
        fusedPosePublisher.set(new Pose3d(swerveDrive.getState().Pose));

        // Debug: project where odometry thinks visible tags are
        Pose3d robotPose3d = new Pose3d(swerveDrive.getState().Pose);
        Transform3d robotToCamera = Constants.kcamToRobot.inverse();
        List<Pose2d> odometryTagPoses = new ArrayList<>();

        for (var result : camera.getCachedResults()) {
            if (!result.hasTargets()) continue;
            for (var target : result.getTargets()) {
                Pose3d estimatedTagPose = robotPose3d
                    .transformBy(robotToCamera)
                    .transformBy(target.getBestCameraToTarget());
                odometryTagPoses.add(estimatedTagPose.toPose2d());
            }
        }
        odometryTagPublisher.set(odometryTagPoses.toArray(new Pose2d[0]));
    }
}