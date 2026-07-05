package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.generated.TunerConstants;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class Vars {
    
    public static double MaxSpeed = 0.7 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double AlignToHubP = 45.0;
    public static final double AlignToHubI = 5.0;
    public static final double AlignToHubD = 0.0;

    public static final double AlignToAllianceWallP = 45.0;
    public static final double AlignToAllianceWallI = 5.0;
    public static final double AlignToAllianceWallD = 0.0;
    // controller stoff

    public final static SlewRateLimiter xLimiter = new SlewRateLimiter(2);
    public final static SlewRateLimiter yLimiter = new SlewRateLimiter(2);
    // no more controller stuph 
    public static double FEED_SPEED = 0.6; // in percentage (0.8 == 80%)
    public static double SHOOTER_SPEED = 30;

    public static final double airTimeScalarSeconds = 1;
    public static final double MaxAngularRate = 1.5;

    public static final double maxYawRateForVision = 6;
}
