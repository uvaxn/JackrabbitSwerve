package frc.robot.commands.AutoAlign;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Variables;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.EaseofLife;
import frc.robot.util.NetworkTables;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class AlignToPoint extends Command {

    private static final double AIM_TOLERANCE_DEGREES = 5.0;

    private final SwerveRequest.FieldCentricFacingAngle request =
        new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withTargetDirectionPerspective(SwerveRequest.TargetDirectionPerspectiveValue.BlueAlliance);

    private final CommandSwerveDrivetrain swerveDrive;
    private final EaseofLife easeOfLife;
    private final Supplier<Translation2d> targetSupplier;

    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier leftSupplier;

    private Rotation2d targetDirection = Rotation2d.kZero;

    // Note: cameraSubsystem is not a dependency here -- alignment uses the
    // drivetrain's continuously-fused pose (odometry + vision, via
    // addVisionMeasurement in Robot.java), not the raw camera reading directly.
    // That fused pose is never "unavailable": it keeps dead-reckoning through
    // brief vision dropouts instead of the align command losing its target the
    // moment a tag glares out or gets briefly occluded.
    //
    // targetSupplier is called fresh every loop in execute(), so a dynamic
    // target (e.g. Landmarks::hubOrNearestShot bound to the current pose) stays
    // correct as the robot moves, not just whatever it was at construction time.
    public AlignToPoint(
            EaseofLife easeOfLife,
            CommandSwerveDrivetrain swerveDrive,
            Supplier<Translation2d> targetSupplier,
            DoubleSupplier forwardSupplier,
            DoubleSupplier leftSupplier) {

        this.easeOfLife = easeOfLife;
        this.swerveDrive = swerveDrive;
        this.targetSupplier = targetSupplier;
        this.forwardSupplier = forwardSupplier;
        this.leftSupplier = leftSupplier;

        request.HeadingController.setPID(
                Variables.AlignToHubP,
                Variables.AlignToHubI,
                Variables.AlignToHubD);

        request.HeadingController.enableContinuousInput(
                -Math.PI,
                Math.PI);
        addRequirements(swerveDrive);
    }

    /** Convenience constructor for a point that never changes. */
    public AlignToPoint(
            EaseofLife easeOfLife,
            CommandSwerveDrivetrain swerveDrive,
            Translation2d target,
            DoubleSupplier forwardSupplier,
            DoubleSupplier leftSupplier) {
        this(easeOfLife, swerveDrive, () -> target, forwardSupplier, leftSupplier);
    }

    /** @return true once the drivetrain's actual heading is within tolerance of the target. */
    public boolean isAimed() {
        final Rotation2d currentHeading = swerveDrive.getState().Pose.getRotation();
        return Math.abs(currentHeading.minus(targetDirection).getDegrees()) < AIM_TOLERANCE_DEGREES;
    }

    @Override
    public void execute() {

        request.HeadingController.setPID(
                easeOfLife.getAlignP(),
                easeOfLife.getAlignI(),
                easeOfLife.getAlignD());

        final double velocityX = forwardSupplier.getAsDouble();
        final double velocityY = leftSupplier.getAsDouble();

        final Pose2d robotPose = swerveDrive.getState().Pose;
        final Translation2d target = targetSupplier.get();
        final Translation2d toTarget = target.minus(robotPose.getTranslation());
        targetDirection = Rotation2d.fromRadians(Math.atan2(toTarget.getY(), toTarget.getX()));

        NetworkTables.putTargetAngle(targetDirection.getDegrees());
        swerveDrive.setControl(
                request
                    .withDeadband(Variables.getMaxSpeed() * 0.1)
                    .withVelocityX(velocityX)
                    .withVelocityY(velocityY)
                    .withTargetDirection(targetDirection));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
