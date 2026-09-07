package frc.robot.constants;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public final class Landmarks {

    private Landmarks() {}
    public static final Translation2d blueHubPosition =
        new Translation2d(
            Units.inchesToMeters(181.56),
            Units.inchesToMeters(158.32));
    public static final Translation2d redHubPosition =
        new Translation2d(
            Units.inchesToMeters(469.11),
            Units.inchesToMeters(158.32));

    // Off-hub points on our own side to shoot at when outside the home box below.
    public static final Translation2d blueUpperShotPosition = new Translation2d(3.551, 5.536);
    public static final Translation2d blueLowerShotPosition = new Translation2d(3.551, 2.424);
    // Red side mirrors blue the same way the hub does: X reflects across the
    // field length (~16.527m, backed out of blueHubPosition/redHubPosition
    // above), Y unchanged. Double check against the real field if exactness matters.
    public static final Translation2d redUpperShotPosition = new Translation2d(12.976, 5.536);
    public static final Translation2d redLowerShotPosition = new Translation2d(12.976, 2.424);

    // Home box: hub is valid to aim at inside it, corners (5.215, 8.200) and (5.215, 0) on blue.
    private static final double HOME_BOX_BLUE_X_MAX = 5.215;
    private static final double HOME_BOX_RED_X_MIN = 11.312;
    private static final double HOME_BOX_Y_MAX = 8.200;

    private static boolean isBlue() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue;
    }

    public static boolean isInHomeBox(Translation2d robotTranslation) {
        double x = robotTranslation.getX();
        double y = robotTranslation.getY();
        boolean withinX = isBlue() ? x <= HOME_BOX_BLUE_X_MAX : x >= HOME_BOX_RED_X_MIN;
        return withinX && y >= 0 && y <= HOME_BOX_Y_MAX;
    }

    /** Whichever off-hub shot point is closer to the given robot position. */
    public static Translation2d nearestShotPosition(Translation2d robotTranslation) {
        Translation2d upper = isBlue() ? blueUpperShotPosition : redUpperShotPosition;
        Translation2d lower = isBlue() ? blueLowerShotPosition : redLowerShotPosition;
        return robotTranslation.getDistance(upper) <= robotTranslation.getDistance(lower) ? upper : lower;
    }

    /** Hub when inside the home box, otherwise whichever off-hub shot point is closer. */
    public static Translation2d hubOrNearestShot(Translation2d robotTranslation) {
        return isInHomeBox(robotTranslation) ? getTeamHubTranslation() : nearestShotPosition(robotTranslation);
    }

    public static Translation2d getTeamHubTranslation() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
            return blueHubPosition;
        }
        return redHubPosition;
    }
}