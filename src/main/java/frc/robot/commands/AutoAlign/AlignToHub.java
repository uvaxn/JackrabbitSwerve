package frc.robot.commands.AutoAlign;

import java.util.function.DoubleSupplier;
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

    /**
     * Pure angle-to-hub math, no side effects -- shared with AlignWhileShooting so the two
     * commands can never drift out of sync on how this is computed.
     */
    public static Rotation2d computeTargetDirection(Pose2d robotPose) {
        Translation2d hub = Constants.getTeamHubTranslation();
        Translation2d toHub = hub.minus(robotPose.getTranslation());
        return Rotation2d.fromRadians(Math.atan2(toHub.getY(), toHub.getX()));
    }

    @Override
    public void initialize() {
        cameraSubsystem.setHubPrecisionMode(true);
    }

    @Override
    public void execute() {

        request.HeadingController.setPID(
                easeOfLife.getAlignP(),
                easeOfLife.getAlignI(),
                easeOfLife.getAlignD());

        double velocityX = forwardSupplier.getAsDouble();
        double velocityY = leftSupplier.getAsDouble();

        // The drivetrain's own pose estimate: wheel odometry, continuously corrected by vision
        // (Limelight.getMeasurement() -> Robot.robotPeriodic() -> swerveDrive.addVisionMeasurement()),
        // using the flat HUB_ALIGN_POS_STD_DEV_M/HUB_ALIGN_ROTATION_STD_DEV std dev while hub
        // precision mode is on (set above in initialize()). This replaces reading
        // cameraSubsystem.getCameraOnlyPose() directly, which also removes the old sim-only
        // fallback branch -- getState().Pose is always populated, in sim and on the real robot
        // alike, so there's no more "no camera pose this cycle" case to handle.
        Pose2d robotPose = swerveDrive.getState().Pose;
        Rotation2d targetDirection = computeTargetDirection(robotPose);

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

    @Override
    public void end(boolean interrupted) {
        cameraSubsystem.setHubPrecisionMode(false);
    }
}