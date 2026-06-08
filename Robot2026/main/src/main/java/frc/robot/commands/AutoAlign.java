package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vars;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CameraSubsystem;

public class AutoAlign extends Command {

    // Cap translation speed during alignment so the robot doesn't outrun its vision
    private final SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric()
        .withDeadband(Vars.AutoAlignMaxSpeed * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final CommandSwerveDrivetrain swerveDrive;
    private final CameraSubsystem cameraSubsystem;
    private final ProfiledPIDController rotationPID;
    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier leftSupplier;

    /**
     * rotates the robot to face the nearest AprilTag
     * @param swerveDrive     the drivetrain subsystem
     * @param cameraSubsystem the vision subsystem
     * @param forwardSupplier field-centric percent max speed (forward)
     * @param leftSupplier    field-centric percent max speed (left)
     */
    public AutoAlign(
        CommandSwerveDrivetrain swerveDrive,
        CameraSubsystem cameraSubsystem,
        DoubleSupplier forwardSupplier,
        DoubleSupplier leftSupplier) {

        this.swerveDrive = swerveDrive;
        this.cameraSubsystem = cameraSubsystem;
        this.forwardSupplier = forwardSupplier;
        this.leftSupplier = leftSupplier;

        rotationPID = new ProfiledPIDController(
            Vars.AlignToHubP,
            Vars.AlignToHubI,
            Vars.AlignToHubD,
            new TrapezoidProfile.Constraints(Math.PI / 2, Math.PI));
        rotationPID.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        swerveDrive.samplePoseAt(Timer.getFPGATimestamp())
            .ifPresent(pose -> rotationPID.reset(pose.getRotation().getRadians()));
    }

    @Override
    public void execute() {
        Optional<Pose2d> possiblePose = swerveDrive.samplePoseAt(Timer.getFPGATimestamp());

        double velocityX = forwardSupplier.getAsDouble() * Vars.AutoAlignMaxSpeed;
        double velocityY = leftSupplier.getAsDouble() * Vars.AutoAlignMaxSpeed;
        double rotationalRate = 0;

        if (possiblePose.isPresent()) {
            Pose2d robotPose = possiblePose.get();
            Optional<Pose2d> nearestTagPose = cameraSubsystem.getNearestTagPose(robotPose);

            if (nearestTagPose.isPresent()) {
                Translation2d toTag = nearestTagPose.get().getTranslation()
                    .minus(robotPose.getTranslation());
                double targetAngle = Math.atan2(toTag.getY(), toTag.getX());

                rotationPID.setGoal(targetAngle);
                rotationalRate = rotationPID.calculate(robotPose.getRotation().getRadians())
                    * Vars.MaxAngularRate;
            }
        }

        swerveDrive.setControl(
            request
                .withVelocityX(velocityX)
                .withVelocityY(velocityY)
                .withRotationalRate(rotationalRate));
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.applyRequest(() -> new SwerveRequest.Idle());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}