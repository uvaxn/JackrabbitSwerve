package frc.robot.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoCenter {
    private CommandSwerveDrivetrain swerve;

    private double kDistToBack = 4; // in meters

    public void AttemptShoot(double dist, double targetAngle) {
        swerve.seedFieldCentric(Rotation2d.kZero);
        
    }
}