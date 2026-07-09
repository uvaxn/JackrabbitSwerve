package frc.robot.vision;
// reminder to press / hold the config button on the limelight GENTLY!

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.Optional;

import com.ctre.phoenix6.SignalLogger;

public class CameraSubsystem extends SubsystemBase {

    private final CommandSwerveDrivetrain swerveDrive;

    private static final String LL_NAME = Constants.LL_NAME;

    private Optional<Pose2d> latestEstimate = Optional.empty();

    private boolean reportedOOB = false;

    private boolean isInField(Translation2d t) {
        return t.getX() >= 0 && t.getX() <= 16.54 &&
            t.getY() >= 0 && t.getY() <= 8.21;
    }
    private final DoubleArrayPublisher visionPosePub =
        NetworkTableInstance.getDefault().getTable("Pose")
            .getDoubleArrayTopic("VisionEstimate").publish();

    public CameraSubsystem(CommandSwerveDrivetrain swerveDrive) {
        this.swerveDrive = swerveDrive;
        LimelightHelpers.SetIMUMode(LL_NAME, 1);
    }

    @Override
    public void periodic() {
        // feed heading (rotation ) to Limelight every loop
        
        LimelightHelpers.SetRobotOrientation(
            LL_NAME,
            swerveDrive.getState().Pose.getRotation().getDegrees(),
            0, 0, 0, 0, 0
        );

        LimelightHelpers.PoseEstimate mt2 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LL_NAME);

    int tagCount = (mt2 == null) ? 0 : mt2.tagCount;
    SignalLogger.writeDouble("Vision/TagCount", tagCount);

    if (mt2 != null && mt2.rawFiducials != null && mt2.rawFiducials.length > 0) {
        double[] seenIds = new double[mt2.rawFiducials.length];
        for (int i = 0; i < mt2.rawFiducials.length; i++) {
            seenIds[i] = mt2.rawFiducials[i].id;
        }
        SignalLogger.writeDoubleArray("Vision/TagIDs", seenIds);
    }

    if (mt2 == null || mt2.tagCount == 0) return;

    // log the raw pose the instant we have one, before any rejection logic runs
    SignalLogger.writeDoubleArray("Vision/RawPose", new double[] {
        mt2.pose.getX(), mt2.pose.getY(), mt2.pose.getRotation().getDegrees()
    });

    if (mt2.tagCount == 1 && mt2.rawFiducials.length > 0 && mt2.rawFiducials[0].ambiguity > 0.5) {
        SignalLogger.writeBoolean("Vision/PoseAccepted", false);
        return;
    }
        Translation2d t = mt2.pose.getTranslation();
        // bounds check
        if (!isInField(t)) {
            DriverStation.reportWarning(
                "Limelight OUT OF BOUNDS POSE (x=" + String.format("%.2f", t.getX()) +
                " y=" + String.format("%.2f", t.getY()) + ")", false);
            SignalLogger.writeBoolean("Vision/PoseAccepted", false);
            return;
        }

        // reject if spinning too fast (megatag 2 no likey that)
        double angularVelocity = Math.abs(swerveDrive.getState().Speeds.omegaRadiansPerSecond);
        if (Math.toDegrees(angularVelocity) > 360) { // tune this threshold
            SignalLogger.writeBoolean("Vision/PoseAccepted", false);
            return;
        }
        latestEstimate = Optional.of(mt2.pose);
        visionPosePub.set(new double[] {
            mt2.pose.getX(), mt2.pose.getY(), mt2.pose.getRotation().getDegrees()
        });

        SignalLogger.writeBoolean("Vision/PoseAccepted", true);
        SignalLogger.writeDoubleArray("Vision/AcceptedPoseXYRotDeg", new double[] {
            mt2.pose.getX(), mt2.pose.getY(), mt2.pose.getRotation().getDegrees()
        });

        // Distance scaled stddevs based on tagsd
        double tagDist = mt2.avgTagDist;
        double stdDev = 0.1 + 0.08 * tagDist * tagDist; // small floor so close up reads aren't treated as perfect
        if (mt2.tagCount > 1) {
            stdDev *= 0.5; 
        }

        SignalLogger.writeDouble("Vision/StdDev", stdDev);
        SignalLogger.writeDouble("Vision/AvgTagDist", tagDist);

        swerveDrive.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds,
            VecBuilder.fill(stdDev, stdDev, 9999999)
        );
        
    }


    public void setLEDBlink() {
    LimelightHelpers.setLEDMode_ForceBlink(LL_NAME);
    }

    public void setLEDNormal() {
        LimelightHelpers.setLEDMode_PipelineControl(LL_NAME);
    }

    public double getDistanceToHub() {
        Optional<Pose2d> pose = getEstimatedPose();
        if (pose.isEmpty()) return 3; // 3 meters by default

        Translation2d hubTarget = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
            ? Constants.redHubPosition
            : Constants.blueHubPosition;

        Translation2d t = pose.get().getTranslation();
        boolean outOfBounds = !isInField(t);

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