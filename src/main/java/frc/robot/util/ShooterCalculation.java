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
            shooterTable.put(2.00, 75.0);
            shooterTable.put(3.00, 85.0);// 3.00 meters | 80 rps
            shooterTable.put(4.00, 87.0);
            shooterTable.put(4.50, 90.5);
            shooterTable.put(5.00, 95.0);// 5.00 meters | 90 rps
        }
        // the reason for the seemingly high rotations per second is bec-
        // because the shooter is 2:1 where 2 motor turns for one flywheel turn.
        public static double calculateShooterSpeed(double distanceMeters) {
            return shooterTable.get(distanceMeters);
        }

}