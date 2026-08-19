package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.generated.TunerConstants;
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

    public final static SlewRateLimiter xLimiter = new SlewRateLimiter(2);
    public final static SlewRateLimiter yLimiter = new SlewRateLimiter(2);
    // no more controller stuph 
    public static double FEED_SPEED = 0.6; // in percentage (0.8 == 80%)
    public static double SHOOTER_SPEED = 30;

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
