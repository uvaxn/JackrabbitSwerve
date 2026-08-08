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

        static {
            shooterTable.put(0.00, 55.0);
            shooterTable.put(1.00, 61.0);
            shooterTable.put(2.00, 67.0);
            shooterTable.put(3.00, 75.0);
            shooterTable.put(4.00, 86.0);
            shooterTable.put(4.50, 95.0); 
        }
        // the reason for the seemingly high rotations per second is because of the fact that the hood is shaped in a way that makes fuel lose a ton of kinetic energy.
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