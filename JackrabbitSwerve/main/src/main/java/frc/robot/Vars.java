package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class Vars {
    public static final Transform3d kcamToRobot = new Transform3d(Units.inchesToMeters(0), Units.inchesToMeters(12), Units.inchesToMeters(1), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)));
    public static final double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double AlignToHubP = 0.35;
    public static final double AlignToHubI = 0.07;
    public static final double AlignToHubD = 0.1;

    public static final double airTimeScalarSeconds = 1.0;
    public static final double MaxAngularRate = 3;

}
