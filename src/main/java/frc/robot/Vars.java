package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.generated.TunerConstants;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class Vars {
    
    public static final double MaxSpeed = 0.4 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double AlignToHubP = 25.0;
    public static final double AlignToHubI = 5.0;
    public static final double AlignToHubD = 0.0;

    public static final double AlignToAllianceWallP = 35.0;
    public static final double AlignToAllianceWallI = 10.0;
    public static final double AlignToAllianceWallD = 0.0;
    // controller stoff

    public final static SlewRateLimiter xLimiter = new SlewRateLimiter(1);
    public final static SlewRateLimiter yLimiter = new SlewRateLimiter(1);
    // no more controller stuph 
    public static double FEED_SPEED = 0.8; // in percentage (0.8 == 80%)
    public static double SHOOTER_SPEED = 30;
    //
    public static final double airTimeScalarSeconds = 7.629; // higher number = less predicition
    public static final double MaxAngularRate = 1;

    public static final double maxYawRateForVision = 6;
}
