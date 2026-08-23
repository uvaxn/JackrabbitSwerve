package frc.robot.commands.AutoAlign;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.TargetDirectionPerspectiveValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Variables;
import frc.robot.constants.Constants;
import frc.robot.constants.LimelightConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.EaseofLife;
import frc.robot.util.NetworkTables;
import frc.robot.vision.Limelight;

/**
 * Runs while a VISION-based firing sequence is spinning up or actively firing -- see
 * Mechanisms.isShootingWithVision(), which is deliberately false during the left-bumper
 * fixed/backup shot so that shot never drags a heading correction along with it -- and picks
 * a target heading every cycle based on where the robot actually is:
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
 * While HUB-facing during autonomous, this also runs the once-per-shot reseed check (see
 * Limelight.checkAutonomousReseed) -- but note the binding below is teleop-gated (see
 * RobotContainer), so today this only actually happens if something explicitly schedules this
 * command during auto. A plain Trigger can't safely do that itself: a PathPlanner auto's
 * SequentialCommandGroup holds drivetrain as a requirement for its entire scheduled lifetime
 * (every sub-command's requirements, unioned, not just whichever step is currently active), so
 * an externally-triggered command racing it for drivetrain would cancel the whole auto, not
 * hand off cleanly. To use this during auto, compose it INSIDE the auto's own command tree
 * instead -- e.g. register this as its own NamedCommand (translation suppliers as () -> 0.0,
 * driving is PathPlanner's job during auto) and wrap the existing shoot/wait/stop-shoot steps
 * in a "deadline" group racing against it, so its requirement is part of the same group from
 * the start rather than an external claim on drivetrain.
 * <p>
 * Bound in RobotContainer to a Trigger that combines the right-trigger shooting state with
 * EaseofLife.isAutoAlignEnabled() (on by default; the Y button used to toggle it but that
 * binding was reassigned to seedFieldCentric, so today this is permanently on unless
 * toggleAutoAlign() gets rewired elsewhere -- see EaseofLife) and DriverStation.isTeleopEnabled()
 * (see above).
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

    // Autonomous-alignment reseed: latches true the one time cameraSubsystem.checkAutonomousReseed
    // actually fires for THIS run of the command, so it can only ever happen once per shot (see
    // initialize(), execute(), and Limelight.checkAutonomousReseed's doc). Only meaningful in the
    // HUB-facing branch below -- reseeding while facing the wall isn't a thing.
    private boolean hasReseeded = false;

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
    public void initialize() {
        hasReseeded = false;
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

            // Autonomous alignment reseed -- same rules as AlignToHub: autonomous only (never
            // fires in teleop, no surprise pose snaps mid-drive), once per run of this command,
            // once we're both close to on-target and the vision pose is stable. This is the
            // path that actually matters for real autos: AlignToHub itself isn't bound to
            // anything, this command is what runs whenever "shoot" fires, in auto or teleop.
            if (DriverStation.isAutonomousEnabled() && !hasReseeded) {
                double headingErrorDegrees = Math.abs(
                        targetDirection.minus(robotPose.getRotation()).getDegrees());

                cameraSubsystem
                        .checkAutonomousReseed(
                                headingErrorDegrees, hasReseeded, LimelightConstants.TRUST_PERCENT_AUTONOMOUS)
                        .ifPresent(reseedPose -> {
                            swerveDrive.resetPose(reseedPose);
                            hasReseeded = true;
                        });
            }
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