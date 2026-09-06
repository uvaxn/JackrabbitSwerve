package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;

public final class Landmarks {

    private Landmarks() {}

    // --- Your own measured hub positions ---
    public static final Translation2d blueHubPosition = Constants.blueHubPosition;
    public static final Translation2d redHubPosition = Constants.redHubPosition;

    /** Alliance-aware hub position. Same behavior as {@link Constants#getTeamHubTranslation()}. */
    public static Translation2d hubPosition() {
        return Constants.getTeamHubTranslation();
    }
}