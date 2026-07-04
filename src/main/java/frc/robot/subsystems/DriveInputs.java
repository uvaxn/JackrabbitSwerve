package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Vars;

import java.util.function.DoubleSupplier;

public class DriveInputs extends SubsystemBase {
    private final DoubleSupplier rawXSupplier;
    private final DoubleSupplier rawYSupplier;

    private final CommandXboxController controller;
    private double limitedX = 0;
    private double limitedY = 0;

    public DriveInputs(DoubleSupplier rawXSupplier, DoubleSupplier rawYSupplier, CommandXboxController controller) {
        this.rawXSupplier = rawXSupplier;
        this.rawYSupplier = rawYSupplier;
        this.controller = controller;
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
    
    public Command rumbleWithDuration(double seconds, double amount) {
        return new SequentialCommandGroup(
            new InstantCommand(() ->
                controller.getHID().setRumble(RumbleType.kBothRumble, amount)),
            new WaitCommand(seconds),
            new InstantCommand(() ->
                controller.getHID().setRumble(RumbleType.kBothRumble, 0))
        );
    }
}