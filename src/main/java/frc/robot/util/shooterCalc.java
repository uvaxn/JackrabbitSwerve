package frc.robot.util;
import frc.robot.subsystems.robot.ShooterSubsystem;
public class shooterCalc {
    
    
    /**
     * Pure distance-to-speed calculations for the shooter, kept separate from
     * {@link ShooterSubsystem} so that file only has to deal with hardware.
     */

    
        private static final double MIN_DIST = 0;
        private static final double MAX_DIST = 4.5;
    
        private shooterCalc() {}
    
        /**
         * Maps a distance to the hub (meters) to a target shooter speed using a power curve
         * between the min/max speed pulled from NetworkTables. {@code exponent} shapes the
         * curve -- 1.0 is linear, greater than 1.0 ramps speed up more sharply as distance
         * increases.
         */
        public static double calculateShooterSpeed(double dist) {
            double minSpeed = nt.getShooterMinSpeed();
            double maxSpeed = nt.getShooterMaxSpeed();
            double exponent = nt.getShooterExponent();
    
            double distance = Math.max(MIN_DIST, Math.min(MAX_DIST, dist));
            double normalized = (distance - MIN_DIST) / (MAX_DIST - MIN_DIST);
    
            return minSpeed + Math.pow(normalized, exponent) * (maxSpeed - minSpeed);
        }

}

