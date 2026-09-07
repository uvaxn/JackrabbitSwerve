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

                public static Translation2d getTeamHubTranslation() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
            return blueHubPosition;
        }
        return redHubPosition;
    }
}