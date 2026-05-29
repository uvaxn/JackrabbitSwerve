package frc.robot.commands;

import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vars;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoAlign extends Command {

    private static final Set<Integer> RED_HUB_TAGS  = Set.of(2, 3, 4, 5, 8, 9, 10, 11);
    private static final Set<Integer> BLUE_HUB_TAGS = Set.of(18, 19, 20, 21, 24, 25, 26, 27);

    private final SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric()
            .withDeadband(Vars.MaxSpeed * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final ProfiledPIDController rotationPID = new ProfiledPIDController(
        Vars.AlignToHubP, Vars.AlignToHubI, Vars.AlignToHubD,
        new TrapezoidProfile.Constraints(Math.PI / 2, Math.PI)
    );

    private final CommandSwerveDrivetrain swerveDrive;
    private final CameraSubsystem cameraSubsystem;
    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier leftSupplier;
    private Translation2d hubCenter;
    private Set<Integer> hubTagIds;

    public AutoAlign(CommandSwerveDrivetrain swerveDrive,
                     CameraSubsystem cameraSubsystem,
                     DoubleSupplier forwardSupplier,
                     DoubleSupplier leftSupplier) {
        this.swerveDrive = swerveDrive;
        this.cameraSubsystem = cameraSubsystem;
        this.forwardSupplier = forwardSupplier;
        this.leftSupplier = leftSupplier;
        addRequirements(swerveDrive);
        rotationPID.enableContinuousInput(-Math.PI, Math.PI);
        rotationPID.setTolerance(Math.toRadians(2));
    }

    @Override
    public void initialize() {
        var alliance = DriverStation.getAlliance();
        hubTagIds = (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red)
            ? RED_HUB_TAGS : BLUE_HUB_TAGS;

        // Compute hub center as fallback
        double totalX = 0, totalY = 0;
        int count = 0;
        for (int id : hubTagIds) {
            var tagPose = cameraSubsystem.getTagPose2d(id);
            if (tagPose.isPresent()) {
                totalX += tagPose.get().getX();
                totalY += tagPose.get().getY();
                count++;
            }
        }
        hubCenter = count > 0
            ? new Translation2d(totalX / count, totalY / count)
            : new Translation2d(0, 0);

        rotationPID.reset(swerveDrive.getState().Pose.getRotation().getRadians());
    }

    @Override
    public void execute() {
        Pose2d robotPose = swerveDrive.getState().Pose;

        // Find nearest visible hub tag
        double bestDist = Double.MAX_VALUE;
        Optional<Translation2d> nearestTag = Optional.empty();

        for (var result : cameraSubsystem.getCachedResults()) {
            if (!result.hasTargets()) continue;
            for (var target : result.getTargets()) {
                int id = target.getFiducialId();
                if (!hubTagIds.contains(id)) continue;
                var tagPose = cameraSubsystem.getTagPose2d(id);
                if (tagPose.isEmpty()) continue;
                double dist = robotPose.getTranslation().getDistance(tagPose.get().getTranslation());
                if (dist < bestDist) {
                    bestDist = dist;
                    nearestTag = Optional.of(tagPose.get().getTranslation());
                }
            }
        }

        // Use nearest tag if visible, otherwise fall back to hub center
        Translation2d target = nearestTag.orElse(hubCenter);
        Translation2d toTarget = target.minus(robotPose.getTranslation());
        double targetAngle = Math.atan2(toTarget.getY(), toTarget.getX());

        rotationPID.setGoal(targetAngle);
        double output = rotationPID.calculate(robotPose.getRotation().getRadians());

        swerveDrive.setControl(
            request.withVelocityX(forwardSupplier.getAsDouble() * Vars.MaxSpeed)
                   .withVelocityY(leftSupplier.getAsDouble() * Vars.MaxSpeed)
                   .withRotationalRate(output * Vars.MaxAngularRate)
        );
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.setControl(new SwerveRequest.Idle());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}