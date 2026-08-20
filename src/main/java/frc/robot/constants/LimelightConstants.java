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
    public static final double MAX_SINGLE_TAG_AMBIGUITY = 0.7;      // was 0.5
    public static final double MAX_ANGULAR_VEL_DEG_PER_SEC = 900.0;  // was 720.0 -- already above
    // MaxAngularRate's physical ceiling (1.5 rot/s = 540 deg/s) either way, so this check was
    // already effectively non-binding; bumped for consistency, don't expect a visible change.
    public static final double MAX_ACCEPT_DIST_M = 8.0;              // was 6.5

    // Lower distance penalty and a looser ceiling
    // instead of being clamped down hard or excluded from swinging the pose estimator.
    public static final double POS_STD_DEV_FLOOR = 0.1;
    public static final double POS_STD_DEV_DIST_COEFF = 0.05;      // was 0.08
    public static final double MULTI_TAG_STD_DEV_SCALE = 0.5;
    public static final double MAX_ACCEPTED_POS_STD_DEV = 6.0;     // was 4.5 (originally 3.0)

    // ---- rotation stddev for the MT1-sourced drift correction ----
    // MT2's own rotation just echoes the gyro heading it was fed, it is NOT an independent
    // measurement, so MT1 (which solves rotation from tag geometry alone) is the only thing
    // here that can actually correct gyro drift. It only gets to do so when trustworthy.
    public static final double MT1_ROTATION_STD_DEV_MULTI_TAG_RAD = 0.3;
    public static final double MT1_ROTATION_STD_DEV_SINGLE_TAG_RAD = 1.0;
    public static final double UNTRUSTED_ROTATION_STD_DEV = 9999999;

    // ---- staleness: how long a fused pose stays valid with nothing refreshing it ----
    public static final double MAX_ESTIMATE_AGE_SECONDS = 0.6;     // was 0.4 (originally 0.25)
    // Getting long for a fast-moving robot -- a stale getDistanceToHub() read this old could
    // meaningfully disagree with where the robot actually is. The left-bumper fixed shot exists
    // partly to backstop exactly this case, but keep an eye on it if shots start missing long/short.

    // ---- pose stability: has vision agreed with where we already thought we were for a
    // while?
    public static final double AGREED_TRANSLATION_EPSILON_M = 0.5; // was 0.35 (originally 0.20)
    public static final int DEFAULT_STABLE_UPDATE_THRESHOLD = 25;   

    // ---- hub precision mode: while AlignToHub/AlignWhileShooting is actively aiming at the
    // HUB, skip the trust-percentage system below entirely in favor of one flat, simpler
    // position std dev -- a deliberately separate override, not expressed as some particular
    // trust percentage (see Limelight.getMeasurement()).
    public static final double HUB_ALIGN_POS_STD_DEV_M = 0.2;

    // ---- vision trust percentage: the knob a driver/mentor actually turns (0-100, see
    // Limelight.setVisionTrustPercent), mapped onto a multiplier for PoseFusion's adaptive
    // fusion.posStdDevBase. 50% is a no-op (1x, today's unmodified behavior); the curve is a
    // logit/odds-ratio shape so the percentage reads as roughly linear "how much do I trust
    // this" while the std dev it produces swings exponentially, matching how a Kalman-style
    // filter actually responds to a trust change. TRUST_CURVE_EXPONENT is *derived*, not
    // hand-tuned: it's the unique power that also lands 25% exactly on 2x, solved from
    // ((1 - 0.25) / 0.25)^p = 2  =>  3^p = 2  =>  p = log(2) / log(3).
    // 100% -> multiplier -> 0 (vision essentially unopposed) and 0% -> multiplier -> infinity,
    // both real limits of this one formula, capped by VISION_TRUST_DISABLED_STD_DEV below.
    public static final double TRUST_CURVE_EXPONENT = Math.log(2.0) / Math.log(3.0);

    // Position std dev used whenever vision trust effectively bottoms out (0% exactly, or any
    // percentage low enough that the curve above would blow past this) -- vision is being
    // "ignored" at this point, not just distrusted a lot.
    public static final double VISION_TRUST_DISABLED_STD_DEV = 999999;

    // Vision is never trusted for rotation through this system, regardless of trust percentage
    // or hub precision mode -- MT2's own rotation just echoes the gyro heading it was fed (not
    // an independent measurement), and even MT1's geometry-solved rotation
    // (fusion.rotationStdDevRad, still computed by PoseFusion, just no longer surfaced here) is
    // deliberately left unused by policy now, not because it stopped working.
    public static final double VISION_ROTATION_STD_DEV = 999999;

    // ---- vision trust percentage per operating mode -- see Robot.java's mode-transition
    // hooks for NORMAL/AUTONOMOUS/TELEOP, and Limelight.checkAutonomousReseed for RESEED ----
    public static final double TRUST_PERCENT_NORMAL = 50.0;
    public static final double TRUST_PERCENT_AUTONOMOUS = 25.0;
    public static final double TRUST_PERCENT_TELEOP = 100.0;
    public static final double TRUST_PERCENT_RESEED = 100.0;

    // ---- autonomous alignment reseed: how close to the target heading counts as "aligned
    // enough to trust a hard pose reset" (see Limelight.checkAutonomousReseed) ----
    // Relaxed less aggressively than the other constants above on purpose: this one gates an
    // instant, irreversible pose overwrite mid-autonomous, not just a routine fusion weight, so
    // being too loose here risks snapping to a "reseed" pose while still meaningfully misaligned.
    public static final double AUTONOMOUS_RESEED_HEADING_TOLERANCE_DEG = 5.0; // was 4.0 (originally 2.0)
}