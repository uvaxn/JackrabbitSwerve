package frc.robot.controls;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Vars;
import java.util.function.DoubleSupplier;

public class DriveInputs extends SubsystemBase {
    private final DoubleSupplier rawXSupplier;
    private final DoubleSupplier rawYSupplier;

    private final SlewRateLimiter speedLimiter =
        new SlewRateLimiter(Vars.maxAccel, -Vars.maxDecel, 0);

    private double lastAngle = 0;
    private double limitedX = 0;
    private double limitedY = 0;

    public DriveInputs(DoubleSupplier rawXSupplier, DoubleSupplier rawYSupplier) {
        this.rawXSupplier = rawXSupplier;
        this.rawYSupplier = rawYSupplier;
    }

    @Override
    public void periodic() {
        double rawX = rawXSupplier.getAsDouble();
        double rawY = rawYSupplier.getAsDouble();
        double stickMag = Math.hypot(rawX, rawY);

        double speed;
        if (stickMag < 0.05) {
            speed = speedLimiter.calculate(0);
        } else {
            double desiredSpeed = stickMag * Vars.MaxSpeed;
            double angle = Math.atan2(rawY, rawX);
            double angleDiff = Math.abs(MathUtil.angleModulus(angle - lastAngle));

            if (angleDiff > Math.toRadians(150)) {
                speed = speedLimiter.calculate(0);
                if (Math.abs(speed) < 0.05) {
                    lastAngle = angle;
                }
            } else {
                speed = speedLimiter.calculate(desiredSpeed);
                lastAngle = angle;
            }
        }

        limitedX = speed * Math.cos(lastAngle);
        limitedY = speed * Math.sin(lastAngle);
    }

    public double getX() { return limitedX; }
    public double getY() { return limitedY; }
}