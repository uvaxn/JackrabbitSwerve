package frc.robot.commands.AutoAlign;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.TargetDirectionPerspectiveValue;

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
            .withTargetDirectionPerspective(TargetDirectionPerspectiveValue.BlueAlliance)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

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
    public void initialize() {
        cameraSubsystem.setAlignMode(true);
    }

    @Override
    public void execute() {

        request.HeadingController.setPID(
                easeOfLife.getAlignP(),
                easeOfLife.getAlignI(),
                easeOfLife.getAlignD());

        double velocityX = forwardSupplier.getAsDouble();
        double velocityY = leftSupplier.getAsDouble();

        // No real Limelight exists under simulation, so getCameraOnlyPose() would never
        // return a value there and this command would be untestable on the desktop sim.
        // Fall back to the drivetrain's own (simulated) pose in that case; on the real
        // robot this reads the camera-only pose exactly as before.
        Optional<Pose2d> alignPose = Utils.isSimulation()
                ? Optional.of(swerveDrive.getState().Pose)
                : cameraSubsystem.getCameraOnlyPose();

        if (alignPose.isPresent()) {

            Pose2d robotPose = alignPose.get();

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

            // No camera pose this cycle.
            // Continue driving while keeping the previous heading target.
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

    @Override
    public void end(boolean interrupted) {
        cameraSubsystem.setAlignMode(false);
    }
}