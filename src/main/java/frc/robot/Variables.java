package frc.robot;

import frc.robot.generated.TunerConstants;
import frc.robot.util.FalloffRateLimiter;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

public class Variables {
    
    public static final double AlignToHubP = 5.0;
    public static final double AlignToHubI = 0;
    public static final double AlignToHubD = 0.5;

    public static final double AlignToAllianceWallP = 7.0;
    public static final double AlignToAllianceWallI = 0.0;
    public static final double AlignToAllianceWallD = 0.25;
    // controller stoff

    // Same feel as MoSim's DriveController falloff curve: quick punch off zero, soft taper
    // near max speed instead of a flat ramp. 2.5 carried over from the old SlewRateLimiter's
    // rate - re-tune once you drive it, it's the "how hard off the line" knob.
    // falloffPercent/falloffExponent are the game's defaults, tune those for how late/soft
    // the top-end taper feels. maxOutput MUST match whatever this limiter's input/output units
    // are - getX()/getY() multiply this limiter's [-1, 1] output by getMaxSpeed() afterward,
    // so maxOutput here stays 1.0, not BASE_SPEED.
    public final static FalloffRateLimiter xLimiter = new FalloffRateLimiter(2.5, 0.075, 10, 1.0);
    public final static FalloffRateLimiter yLimiter = new FalloffRateLimiter(2.5, 0.075, 10, 1.0);
    // no more controller stuph 
    public static double FEED_SPEED = 0.6; // in percentage (0.8 == 80%)
    public static double SHOOTER_SPEED = 30;
    // Left-bumper backup shot -- bypasses vision/distance calc entirely, see
    // ShooterSubsystem.startFixed(). Placeholder pulled from the middle of
    // ShooterCalculation's table (roughly the 2m rung); tune this to wherever
    // you actually plan to take this shot from. Live-tunable on the dashboard
    // too, see NetworkTables.getFixedShooterSpeed().
    public static double FIXED_SHOOTER_SPEED = 70;

    // Rough average field-relative speed the fuel leaves the shooter at, in m/s. Used ONLY to
    // lead-compensate AlignToHub/AlignWhileShooting's aim direction for the robot's own motion
    // (see AlignToHub.computeTargetDirection) -- this is NOT the same number as
    // ShooterCalculation's RPS table. That table is flywheel surface speed, and this hood
    // design loses a lot of that before it reaches the fuel (see ShooterCalculation's own
    // comment on that), so RPS * wheel circumference would overstate the real exit speed.
    // This is a single placeholder average across all distances instead of a real measurement.
    // To tune it: strafe at a known speed while aiming at the HUB and adjust this number until
    // stationary shots land dead-on instead of drifting toward whichever way you were moving.
    // Live-tunable on the dashboard too, see NetworkTables.getFuelExitSpeedMps().
    public static double FUEL_EXIT_SPEED_MPS = 9.0;

    public static final double airTimeScalarSeconds = 1;
    public static final double MaxAngularRate = 1.5;

    public static final double maxYawRateForVision = 6;


    private static final double BASE_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private static final double DEFAULT_SPEED_MULTIPLIER = 0.7;

    // Named speed-limit requests. Effective MaxSpeed = BASE_SPEED * (min of all active requests).
    private static final Map<String, Double> speedLimitRequests = new ConcurrentHashMap<>();

    public static void requestSpeedLimit(String source, double multiplier) { // Added in purely for ShooterSubsystem. If another file wants to change the maxspeed, this way is much better than casting changes over files.
        speedLimitRequests.put(source, multiplier);
    }


    public static void clearSpeedLimit(String source) {
        speedLimitRequests.remove(source);
    }

    public static double getMaxSpeed() {
        double multiplier = speedLimitRequests.values().stream()
            .min(Double::compare)
            .orElse(DEFAULT_SPEED_MULTIPLIER);
        return multiplier * BASE_SPEED;
    }
    
}