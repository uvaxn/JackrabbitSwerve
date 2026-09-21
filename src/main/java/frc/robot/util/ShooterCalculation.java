package frc.robot.util;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
public class ShooterCalculation {
        private ShooterCalculation() {}
    
        private static final InterpolatingDoubleTreeMap shooterTable =
                new InterpolatingDoubleTreeMap();
        // The more values put, the better.
        // 7 values forr now.d
        // TODO: Manually set the RPS, and once it looks good, put it in here.
        static {
            shooterTable.put(0.00, 47.0);// 0.00 meters | 50 rps
            shooterTable.put(1.00, 50.0);
            shooterTable.put(2.00, 57.0);
            shooterTable.put(3.00, 67.0);// 3.00 meters | 67 rps
            shooterTable.put(4.00, 75.0);
            shooterTable.put(4.50, 85.5);
            shooterTable.put(5.00, 99.0);// 5.00 meters | 99 rps
        }
        // the reason for the seemingly high rotations per second is bec-
        // because the shooter is 2:1 where 2 motor turns for one flywheel turn.
        public static double calculateShooterSpeed(double distanceMeters) {
            return shooterTable.get(distanceMeters);
        }

}