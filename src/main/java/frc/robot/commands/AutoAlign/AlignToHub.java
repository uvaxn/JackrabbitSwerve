package frc.robot.commands.AutoAlign;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Variables;
import frc.robot.constants.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.EaseofLife;
import frc.robot.util.NetworkTables;
import frc.robot.vision.Limelight;
public class AlignToHub extends Command {

    private final SwerveRequest.FieldCentricFacingAngle request =
        new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withTargetDirectionPerspective(SwerveRequest.TargetDirectionPerspectiveValue.BlueAlliance);

    private final Limelight cameraSubsystem;
    private final CommandSwerveDrivetrain swerveDrive;
    private final EaseofLife easeOfLife;

    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier leftSupplier;

    public AlignToHub(
            Limelight cameraSubsystem,
            EaseofLife easeOfLife,
            CommandSwerveDrivetrain swerveDrive,
            DoubleSupplier forwardSupplier,
            DoubleSupplier leftSupplier) {

        this.cameraSubsystem = cameraSubsystem;
        this.easeOfLife = easeOfLife;
        this.swerveDrive = swerveDrive;
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
    @Override
    public void execute() {

        request.HeadingController.setPID(
                easeOfLife.getAlignP(),
                easeOfLife.getAlignI(),
                easeOfLife.getAlignD());

        double velocityX = forwardSupplier.getAsDouble();
        double velocityY = leftSupplier.getAsDouble();
        Optional<Pose2d> cameraPose = cameraSubsystem.getEstimatedPose();
        if (cameraPose.isPresent()) {

            Pose2d robotPose = cameraPose.get();
            Translation2d hub = Constants.getTeamHubTranslation();
            Translation2d toHub = hub.minus(robotPose.getTranslation());
            Rotation2d targetDirection =
                    Rotation2d.fromRadians(
                            Math.atan2(toHub.getY(), toHub.getX()));
            NetworkTables.putTargetAngle(targetDirection.getDegrees());
            swerveDrive.setControl(
                    request
                        .withDeadband(Variables.getMaxSpeed() * 0.1)
                        .withVelocityX(velocityX)
                        .withVelocityY(velocityY)
                        .withTargetDirection(targetDirection));
        } else {
            swerveDrive.setControl(
                    request
                        .withDeadband(Variables.getMaxSpeed() * 0.1)
                        .withVelocityX(velocityX)
                        .withVelocityY(velocityY));
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}