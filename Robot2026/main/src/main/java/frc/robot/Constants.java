package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public class Constants {
    public static final double CAMERA_TILT = Units.degreesToRadians(20);
    public static final double CAMERA_ELEVATION = Units.inchesToMeters(21);
    public static final Transform3d kRobotToCam = new Transform3d(Units.inchesToMeters(0), Units.inchesToMeters(12), Units.inchesToMeters(1), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-20), Units.degreesToRadians(0)));
    public static final Translation2d redHubPosition = 
        new Translation2d(Units.inchesToMeters(492.33), Units.inchesToMeters(158.32));
    public static final Translation2d blueHubPosition = 
        new Translation2d(Units.inchesToMeters(158.34), Units.inchesToMeters(158.32));
    public static Translation2d getTeamHubTranslation() {
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            return blueHubPosition;
        }
        return redHubPosition;
    }
}
