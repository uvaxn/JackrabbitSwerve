package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.generated.TunerConstants;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class Vars {
    
    public static final double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double AlignToHubP = 25.0;
    public static final double AlignToHubI = 5.0;
    public static final double AlignToHubD = 0.5;
    // controller stoff
    public static final SlewRateLimiter xLimiter   = new SlewRateLimiter(7.0, -7.0, 0);
    public static final SlewRateLimiter yLimiter   = new SlewRateLimiter(7.0, -7.0, 0);
    public static double lastLimitedX = 0;
    public static double lastLimitedY = 0;
    // no more controller stuph

    // intake

    // shooters
    public static double SHOOTER_SPEED = 0.8;
    public static double FEED_SPEED = 0.8;
    //
    public static final double airTimeScalarSeconds = 7.629; // higher number = less predicition
    public static final double MaxAngularRate = 1;
    public static final double maxYawRateForVision = 0;
}
