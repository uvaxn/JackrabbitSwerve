package frc.robot.constants;


import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
// added import
import java.util.Optional;


public class Constants {
    public static final String LL_NAME = "limelight-one";
    public static final Transform3d ROBOT_TO_CAM = new Transform3d(Units.inchesToMeters(0), Units.inchesToMeters(12), Units.inchesToMeters(1), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-20), Units.degreesToRadians(0)));
    public static final Translation2d blueHubPosition =
        new Translation2d(
            Units.inchesToMeters(182.04724),
            Units.inchesToMeters(158.77953));

            
    public static final Translation2d redHubPosition =
        new Translation2d(
            Units.inchesToMeters(469.015748),
            Units.inchesToMeters(158.77953));
    public static Translation2d getTeamHubTranslation() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
            return blueHubPosition;
        }
        return redHubPosition;
        // defaults to redhubPOS
    }

    // ---- Alliance zones (blue-origin field coords, verified against the 2026 REBUILT field:
    // ~16.54m x ~8.07m overall, rulebook ALLIANCE ZONE is 158.6in/~4.03m deep -- these corners
    // extend that depth by the TRENCH, ~47in/~1.19m, so "own zone" means "at or before our own
    // trench", not just the bare rulebook zone). Used by AlignWhileShooting to decide whether
    // we're close enough to face the HUB, or should just face our own alliance wall instead.
    public static final Translation2d BLUE_ALLIANCE_ZONE_CORNER_1 = new Translation2d(0.0, 0.0);
    public static final Translation2d BLUE_ALLIANCE_ZONE_CORNER_2 = new Translation2d(5.215, 8.104);
    public static final Translation2d RED_ALLIANCE_ZONE_CORNER_1 = new Translation2d(11.342, 0.0);
    public static final Translation2d RED_ALLIANCE_ZONE_CORNER_2 = new Translation2d(16.561, 8.104);

    /**
     * @param position a field position in meters, blue-origin field coordinates
     *                  (e.g. drivetrain.getState().Pose.getTranslation())
     * @return true if position is inside the current alliance's own alliance zone.
     * Defaults to the red zone if alliance color isn't known yet, matching getTeamHubTranslation().
     */
    public static boolean isInOwnAllianceZone(Translation2d position) {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        boolean isBlue = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue;

        Translation2d corner1 = isBlue ? BLUE_ALLIANCE_ZONE_CORNER_1 : RED_ALLIANCE_ZONE_CORNER_1;
        Translation2d corner2 = isBlue ? BLUE_ALLIANCE_ZONE_CORNER_2 : RED_ALLIANCE_ZONE_CORNER_2;

        double minX = Math.min(corner1.getX(), corner2.getX());
        double maxX = Math.max(corner1.getX(), corner2.getX());
        double minY = Math.min(corner1.getY(), corner2.getY());
        double maxY = Math.max(corner1.getY(), corner2.getY());

        return position.getX() >= minX && position.getX() <= maxX
            && position.getY() >= minY && position.getY() <= maxY;
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