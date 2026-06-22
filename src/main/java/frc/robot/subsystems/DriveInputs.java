package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Vars;

import java.util.function.DoubleSupplier;

public class DriveInputs extends SubsystemBase {
    private final DoubleSupplier rawXSupplier;
    private final DoubleSupplier rawYSupplier;

    private double limitedX = 0;
    private double limitedY = 0;

    public DriveInputs(DoubleSupplier rawXSupplier, DoubleSupplier rawYSupplier) {
        this.rawXSupplier = rawXSupplier;
        this.rawYSupplier = rawYSupplier;
    }

    @Override
    public void periodic() {
        double rawX = MathUtil.applyDeadband(
            rawXSupplier.getAsDouble(), 0.08);

        double rawY = MathUtil.applyDeadband(
            rawYSupplier.getAsDouble(), 0.08);

        limitedX = Vars.xLimiter.calculate(rawX);
        limitedY = Vars.yLimiter.calculate(rawY);
    }

    public double getX() {
        return limitedX * Vars.MaxSpeed;
    }

    public double getY() {
        return limitedY * Vars.MaxSpeed;
    }
}