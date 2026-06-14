package frc.robot.vision;
/* 
TODO: alot of things, first off flash LimeLightOS 4 onto the camera, set the translation in the settings 
found inside constants, Set the correct field map upload the AndyMark fmap,  Tune maxYawRateForVision in 
Vars, and ensure you have a good name for the limelight.
 */ 

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Vars;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.Optional;

public class CameraSubsystem extends SubsystemBase {

    private final CommandSwerveDrivetrain swerveDrive;

    private Optional<Pose2d> latestEstimate = Optional.empty();

    public CameraSubsystem(CommandSwerveDrivetrain swerveDrive) {
        this.swerveDrive = swerveDrive;
        LimelightHelpers.SetIMUMode("limelight", 1);
    }

    @Override
    public void periodic() {
        // feed heading (rotation ) to Limelight every loop 
        
        LimelightHelpers.SetRobotOrientation(
            "limelight",
            swerveDrive.getState().Pose.getRotation().getDegrees(),
            0, 0, 0, 0, 0
        );

        LimelightHelpers.PoseEstimate mt2 =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        if (mt2 == null || mt2.tagCount == 0) return;

        latestEstimate = Optional.of(mt2.pose);

        Translation2d t = mt2.pose.getTranslation();

        // bounds check
        if (t.getX() < 0 || t.getX() > 16.54 ||
            t.getY() < 0 || t.getY() > 8.21) {
            DriverStation.reportWarning(
                "Limelight OUT OF BOUNDS POSE (x=" + String.format("%.2f", t.getX()) +
                " y=" + String.format("%.2f", t.getY()) + ")", false);
            return;
        }

        // reject if spinning too fast (megatag 2 no likey that)
        if (Math.abs(swerveDrive.getState().Speeds.omegaRadiansPerSecond) > Vars.maxYawRateForVision) return;

        // Distance scaled stddevs
        double dist = getDistanceToHub();
        double stdDev = 0.5 * dist * dist;

        swerveDrive.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds,
            VecBuilder.fill(stdDev, stdDev, 9999999)
        );
    }
    public double getDistanceToHub() {
        Optional<Pose2d> pose = swerveDrive.samplePoseAt(Timer.getFPGATimestamp());
        if (pose.isEmpty()) return 3;

    Translation2d hubTarget = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
        ? Constants.redHubPosition
        : Constants.blueHubPosition;

        Pose2d robotPose = pose.get();
        Translation2d t = robotPose.getTranslation();

        if (t.getX() < 0 || t.getX() > 16.54 ||
            t.getY() < 0 || t.getY() > 8.21) {
            DriverStation.reportWarning(
                "getDistanceToHub() OUT OF BOUNDS POSE (x=" + String.format("%.2f", t.getX()) +
                " y=" + String.format("%.2f", t.getY()) + ")", false);
            return 3;
        }

        return hubTarget.minus(t).getNorm();
    }

    // but now in a pose2d format!
    public Optional<Pose2d> getEstimatedPose() {
        return latestEstimate;
    }
}