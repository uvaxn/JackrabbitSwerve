package frc.robot.constants;


import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public class Constants {
    public static final double CAMERA_TILT = Units.degreesToRadians(20);
    public static final String LL_NAME = "limelight";
    public static final double CAMERA_ELEVATION = Units.inchesToMeters(21);
    public static final Transform3d ROBOT_TO_CAM = new Transform3d(Units.inchesToMeters(0), Units.inchesToMeters(12), Units.inchesToMeters(1), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-20), Units.degreesToRadians(0)));
    public static final Translation2d redHubPosition = 
        new Translation2d(Units.inchesToMeters(574.01575), Units.inchesToMeters(158.32));
    public static final Translation2d blueHubPosition = 
        new Translation2d(Units.inchesToMeters(172.2835), Units.inchesToMeters(158.32));
    public static Translation2d getTeamHubTranslation() {
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            return blueHubPosition;
        }
        return redHubPosition;
    }

    // Motors
    public final static int m_ShooterR   = 23;
    public final static int m_ShooterL   = 22;
    public final static int m_Intake     = 25;
    public final static int m_IntakeDrop = 20;
    public final static int m_LowerFeed  = 21;
    public final static int m_UpperFeed  = 24;

    // The rest (drive and steer motors) are best if I just leave them inside 'generated/TunerConstants.'
}
