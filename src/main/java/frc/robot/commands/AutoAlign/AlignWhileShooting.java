package frc.robot.commands.AutoAlign;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.TargetDirectionPerspectiveValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Variables;
import frc.robot.constants.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.EaseofLife;
import frc.robot.util.NetworkTables;
import frc.robot.vision.Limelight;

/**
 * Runs while a firing sequence is spinning up or actively firing, in either feed mode -- see
 * Mechanisms.isShooting() -- and picks a target heading every cycle based on where the robot
 * actually is:
 * <ul>
 *   <li>inside our own alliance zone (Constants.isInOwnAllianceZone) -&gt; face the HUB, same
 *       math as AlignToHub, odometry fused with the flat HUB_ALIGN_* vision std devs</li>
 *   <li>anywhere else, including standing in the opposing alliance's own zone -&gt; face our
 *       own alliance wall instead, same math as AlignToAllianceWall, odometry only</li>
 * </ul>
 * Position is re-checked every execute() cycle rather than decided once at schedule time, so
 * driving across the zone boundary mid-shot switches targets instead of committing to
 * whichever was true the instant the trigger was first pulled. The angle math itself is not
 * duplicated here -- both branches call the same static computeTargetDirection() helpers that
 * AlignToHub / AlignToAllianceWall use on their own, so retuning one place retunes both.
 * <p>
 * Bound in RobotContainer to a Trigger that combines the right-trigger shooting state with
 * EaseofLife.isAutoAlignEnabled() (toggled by the Y button, on by default, published to
 * NetworkTables as Info/AlignMode).
 */
public class AlignWhileShooting extends Command {

    private final SwerveRequest.FieldCentricFacingAngle request =
        new SwerveRequest.FieldCentricFacingAngle()
            .withTargetDirectionPerspective(TargetDirectionPerspectiveValue.BlueAlliance)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Limelight cameraSubsystem;
    private final EaseofLife easeOfLife;
    private final CommandSwerveDrivetrain swerveDrive;

    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier leftSupplier;

    public AlignWhileShooting(
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

        // Seed with the HUB gains; execute() picks the right ones every cycle regardless, this
        // just avoids one cycle of stale/zeroed gains before the first execute() runs.
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

        Pose2d robotPose = swerveDrive.getState().Pose;
        boolean inOwnZone = Constants.isInOwnAllianceZone(robotPose.getTranslation());

        // Only the HUB-facing branch wants the Limelight's flat, less-careful vision std devs
        // (LimelightConstants.HUB_ALIGN_*) blended into the pose estimate -- wall-facing is
        // odometry only, same as AlignToAllianceWall on its own.
        cameraSubsystem.setHubPrecisionMode(inOwnZone);

        Rotation2d targetDirection;
        if (inOwnZone) {
            request.HeadingController.setPID(
                    easeOfLife.getAlignP(),
                    easeOfLife.getAlignI(),
                    easeOfLife.getAlignD());
            targetDirection = AlignToHub.computeTargetDirection(robotPose);
        } else {
            request.HeadingController.setPID(
                    easeOfLife.getAlignWallP(),
                    easeOfLife.getAlignWallI(),
                    easeOfLife.getAlignWallD());
            targetDirection = AlignToAllianceWall.computeTargetDirection();
        }

        NetworkTables.putTargetAngle(targetDirection.getDegrees());

        swerveDrive.setControl(
                request
                        .withDeadband(Variables.getMaxSpeed() * 0.1)
                        .withVelocityX(forwardSupplier.getAsDouble())
                        .withVelocityY(leftSupplier.getAsDouble())
                        .withTargetDirection(targetDirection));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Safe to call unconditionally even if we ended mid wall-facing branch, where it was
        // already false.
        cameraSubsystem.setHubPrecisionMode(false);
    }
}