package frc.robot.commands.AutoAlign;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.TargetDirectionPerspectiveValue;

import edu.wpi.first.math.geometry.Rotation2d;
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
            .withTargetDirectionPerspective(TargetDirectionPerspectiveValue.BlueAlliance)
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

    /**
     * Pure angle math, no side effects -- shared with AlignWhileShooting so the two commands
     * can never drift out of sync on how this is computed. Alliance-color only, no field
     * position involved, so this (and therefore AlignToAllianceWall as a whole) is odometry
     * only -- there's nothing here for vision to correct.
     */
    public static Rotation2d computeTargetDirection() {
        return Rotation2d.fromRadians(
            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                ? 0.0
                : Math.PI);
    }

    @Override
    public void execute() {

        request.HeadingController.setPID(
            easeOfLife.getAlignWallP(),
            easeOfLife.getAlignWallI(),
            easeOfLife.getAlignWallD()
        );

        Rotation2d targetDirection = computeTargetDirection();

        NetworkTables.putTargetAngle(targetDirection.getDegrees());

        swerveDrive.setControl(
            request
                .withDeadband(Variables.getMaxSpeed() * 0.1)
                .withVelocityX(forwardSupplier.getAsDouble())
                .withVelocityY(leftSupplier.getAsDouble())
                .withTargetDirection(targetDirection)
        );
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}