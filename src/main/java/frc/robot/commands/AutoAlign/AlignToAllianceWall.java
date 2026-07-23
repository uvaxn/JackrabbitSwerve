package frc.robot.commands.AutoAlign;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Variables;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.EaseofLife;
import frc.robot.util.NetworkTables;


public class AlignToAllianceWall extends Command {

    private final SwerveRequest.FieldCentricFacingAngle request =
        new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final CommandSwerveDrivetrain swerveDrive;
    private final EaseofLife easeOfLife;

    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier leftSupplier;

    public AlignToAllianceWall(
            CommandSwerveDrivetrain swerveDrive,
            EaseofLife easeOfLife,
            DoubleSupplier forwardSupplier,
            DoubleSupplier leftSupplier) {

        this.swerveDrive = swerveDrive;
        this.easeOfLife = easeOfLife;
        this.forwardSupplier = forwardSupplier;
        this.leftSupplier = leftSupplier;

        request.HeadingController.setPID(
            Variables.AlignToAllianceWallP,
            Variables.AlignToAllianceWallI,
            Variables.AlignToAllianceWallD
        );
        request.HeadingController.enableContinuousInput(
            -Math.PI,
            Math.PI
        );
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {

        request.HeadingController.setPID(
            easeOfLife.getAlignWallP(),
            easeOfLife.getAlignWallI(),
            easeOfLife.getAlignWallD()
        );
        double targetAngle = Math.PI;


        NetworkTables.putTargetAngle(
            Units.radiansToDegrees(targetAngle)
        );

        swerveDrive.setControl(
            request
                .withDeadband(Variables.getMaxSpeed() * 0.1)
                .withVelocityX(forwardSupplier.getAsDouble())
                .withVelocityY(leftSupplier.getAsDouble())
                .withTargetDirection(
                    Rotation2d.fromRadians(targetAngle)
                )
        );
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}