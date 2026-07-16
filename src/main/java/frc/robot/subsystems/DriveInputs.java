package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Variables;

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

        limitedX = Variables.xLimiter.calculate(rawX);
        limitedY = Variables.yLimiter.calculate(rawY);
    }

    public double getX() {
        return limitedX * Variables.getMaxSpeed();
    }

    public double getY() {
        return limitedY * Variables.getMaxSpeed();
    }
    

    public Command rumblePulse(int pulses, double pulseLength, double timeBetweenPulses, double amount) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        

        for (int i = 0; i < pulses; i++) {
            command.addCommands(
                new InstantCommand(() ->
                    controller.getHID().setRumble(RumbleType.kBothRumble, amount)
                ),
                new WaitCommand(pulseLength),
                new InstantCommand(() ->
                    controller.getHID().setRumble(RumbleType.kBothRumble, 0)
                )
            );
            if (i < pulses - 1) {
                command.addCommands(new WaitCommand(timeBetweenPulses));
            }
        }

        return command;
    }
}