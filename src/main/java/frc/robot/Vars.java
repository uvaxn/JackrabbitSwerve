package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.generated.TunerConstants;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class Vars {
    
    public static final double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double AlignToHubP = 25.0;
    public static final double AlignToHubI = 5.0;
    public static final double AlignToHubD = 0.0;

    public static final double AlignToAllianceWallP = 35.0;
    public static final double AlignToAllianceWallI = 10.0;
    public static final double AlignToAllianceWallD = 0.0;
    // controller stoff

    public static final double maxAccel = 3.0;  // m/s ^2 
    public static final double maxDecel = 10.0; // m/s ^2
    public static final SlewRateLimiter speedLimiter = new SlewRateLimiter(maxAccel, -maxDecel, 0);
    public static double lastDriveAngle = 0;
    // no more controller stuph 
    
    // shooters
    public static double SHOOTER_SPEED = 20; // in rps
    public static double FEED_SPEED = 0.8; // in percentage
    //
    public static final double airTimeScalarSeconds = 7.629; // higher number = less predicition
    public static final double MaxAngularRate = 1;
    public static final double maxYawRateForVision = 1;
}
