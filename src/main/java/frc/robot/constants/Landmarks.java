package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;

public final class Landmarks {

    private Landmarks() {}
    public static final Translation2d blueHubPosition = Constants.blueHubPosition;
    public static final Translation2d redHubPosition = Constants.redHubPosition;

    public static Translation2d hubPosition() {
        return Constants.getTeamHubTranslation();
    }
}