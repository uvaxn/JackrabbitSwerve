package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

/**
 * Every tunable number the vision stack uses, in one place on purpose: this is the file to
 * open when something needs retuning on the field, nobody should have to go hunting through
 * PoseFusion.java or Limelight.java to find a magic number.
 */
public final class LimelightConstants {
    private LimelightConstants() {}

    private static final AprilTagFieldLayout FIELD =
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    // ---- field bounds, used to sanity check that a pose isn't off in nowhere ----
    public static final double FIELD_LENGTH_M = FIELD.getFieldLength();
    public static final double FIELD_WIDTH_M = FIELD.getFieldWidth();

    // ---- rejection tunables ----
    public static final double MAX_SINGLE_TAG_AMBIGUITY = 0.32;
    public static final double MAX_ANGULAR_VEL_DEG_PER_SEC = 540.0;
    public static final double MAX_ACCEPT_DIST_M = 5.0;

    // ---- position (x/y) stddev model, scaled by MT2 distance/tag count ----
    public static final double POS_STD_DEV_FLOOR = 0.1;
    public static final double POS_STD_DEV_DIST_COEFF = 0.08;
    public static final double MULTI_TAG_STD_DEV_SCALE = 0.5;
    public static final double MAX_ACCEPTED_POS_STD_DEV = 3.0;

    // ---- rotation stddev for the MT1-sourced drift correction ----
    // MT2's own rotation just echoes the gyro heading it was fed, it is NOT an independent
    // measurement, so MT1 (which solves rotation from tag geometry alone) is the only thing
    // here that can actually correct gyro drift. It only gets to do so when trustworthy.
    public static final double MT1_ROTATION_STD_DEV_MULTI_TAG_RAD = 0.3;
    public static final double MT1_ROTATION_STD_DEV_SINGLE_TAG_RAD = 1.0;
    public static final double UNTRUSTED_ROTATION_STD_DEV = 9999999;

    // ---- staleness: how long a fused pose stays valid with nothing refreshing it ----
    public static final double MAX_ESTIMATE_AGE_SECONDS = 0.25;

    // ---- pose stability: has vision agreed with where we already thought we were for a
    // while?
    // (e.g. don't start a scoring sequence off a vision pose that only just showed up).
    public static final double AGREED_TRANSLATION_EPSILON_M = 0.20;
    public static final int DEFAULT_STABLE_UPDATE_THRESHOLD = 50; // ~1s at a 50Hz robotPeriodic()

    // ---- align mode: trust vision position more while a precision alignment command is running
    public static final double ALIGN_STD_DEV_SCALE = 1.0 / 3.0;
}