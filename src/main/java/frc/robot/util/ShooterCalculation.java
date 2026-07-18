package frc.robot.util;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.subsystems.robot.ShooterSubsystem;
public class ShooterCalculation {
    
    
    /**
     * Pure distance-to-speed calculations for the shooter, kept separate from
     * {@link ShooterSubsystem} so that file only has to deal with hardware.
     */
    
        private ShooterCalculation() {}
    
        private static final InterpolatingDoubleTreeMap shooterTable =
                new InterpolatingDoubleTreeMap();
        // The more values put, the better.
        // 7 values forr now.
        // TODO: Manually set the RPS, and once it looks good, put it in here.
        static {
            shooterTable.put(0.00, 55.0);// 0.00 meters | 55 rps
            shooterTable.put(1.00, 60.0);
            shooterTable.put(2.00, 68.0);
            shooterTable.put(3.00, 80.0);// 3.00 meters | 70 rps
            shooterTable.put(4.00, 85.0);
            shooterTable.put(4.50, 87.5);
            shooterTable.put(5.00, 90.0);// 5.00 meters | 85 rps
        }
        // the reason for the seemingly high rotations per second is because of the fact that the hood is shaped in a way that makes energy lose a ton of kinetic energy.
        /**
         * Returns the interpolated shooter speed (RPS) for the given distance.
         *
         * @param distanceMeters Distance to the target in meters.
         * @return Shooter RPS.
         */
        public static double calculateShooterSpeed(double distanceMeters) {
            return shooterTable.get(distanceMeters);
        }

}

