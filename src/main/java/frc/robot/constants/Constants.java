package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
// added import
import java.util.Optional;


public class Constants {
    public static final String LL_NAME = "limelight-one";
    public static final Translation2d blueHubPosition =
        new Translation2d(
            Units.inchesToMeters(181.56),
            Units.inchesToMeters(158.32));
    public static final Translation2d redHubPosition =
        new Translation2d(
            Units.inchesToMeters(469.11),
            Units.inchesToMeters(158.32));

    public static Translation2d getTeamHubTranslation() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
            return blueHubPosition;
        }
        return redHubPosition;
        // defaults to redhubPOS
    }

    public final static int m_ShooterR   = 23;
    public final static int m_ShooterL   = 22;
    public final static int m_Intake     = 25;
    public final static int m_IntakeDrop = 20;
    public final static int m_LowerFeed  = 21;
    public final static int m_UpperFeed  = 24;

}
