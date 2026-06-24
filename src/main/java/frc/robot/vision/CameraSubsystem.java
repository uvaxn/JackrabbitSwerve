package frc.robot.vision;
// reminder to press / hold the config button on the limelight GENTLY!

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

        // Distance scaled stddevs based on tagsd
        double tagDist = mt2.avgTagDist;
        double stdDev = 0.1 + 0.5 * tagDist * tagDist; // small floor so close-up reads aren't treated as perfect
        if (mt2.tagCount > 1) {
            stdDev *= 0.5; // multi-tag solves are more trustworthy
        }
        swerveDrive.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds,
            VecBuilder.fill(stdDev, stdDev, 9999999)
        );
    }
    private boolean reportedOOB = false;

    public double getDistanceToHub() {
        Optional<Pose2d> pose = swerveDrive.samplePoseAt(Timer.getFPGATimestamp());
        if (pose.isEmpty()) return 3;

        Translation2d hubTarget = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
            ? Constants.redHubPosition
            : Constants.blueHubPosition;

        Translation2d t = pose.get().getTranslation();
        boolean outOfBounds = t.getX() < 0 || t.getX() > 16.54 ||
                            t.getY() < 0 || t.getY() > 8.21;

        if (outOfBounds) {
            if (!reportedOOB) {
                DriverStation.reportWarning(
                    "getDistanceToHub() OUT OF BOUNDS POSE (x=" + String.format("%.2f", t.getX()) +
                    " y=" + String.format("%.2f", t.getY()) + ")", false);
                reportedOOB = true; // only logs once, until it recovers
            }
            return 3;
        }
        reportedOOB = false;
        return hubTarget.minus(t).getNorm();
    }

    // but now in a pose2d format!
    public Optional<Pose2d> getEstimatedPose() {
        return latestEstimate;
    }
}