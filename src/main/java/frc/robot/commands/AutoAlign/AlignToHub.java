package frc.robot.commands.AutoAlign;

import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.TargetDirectionPerspectiveValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Variables;
import frc.robot.constants.Constants;
import frc.robot.constants.LimelightConstants;
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

    // latches true the one time cameraSubsystem.checkAutonomousReseed
    // actually fires for THIS command instance, so it can only ever happen once per alignment
    // command (see initialize(), execute(), and Limelight.checkAutonomousReseed's doc).
    private boolean hasReseeded = false;

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
    public static Rotation2d computeTargetDirection(Pose2d robotPose) {
        Translation2d hub = Constants.getTeamHubTranslation();
        Translation2d toHub = hub.minus(robotPose.getTranslation());
        return Rotation2d.fromRadians(Math.atan2(toHub.getY(), toHub.getX()));
    }

    /**
     * "Shoot on the move" lead compensation. The note leaves the shooter carrying the robot's
     * own field-relative velocity along with it -- same idea as a ball thrown from a moving
     * car keeping the car's speed -- so aiming straight at the HUB's real position drifts the
     * shot toward whichever way the robot is currently traveling. Instead, this aims the shot's
     * RELATIVE-to-robot velocity so that once the robot's own velocity adds back on top of it
     * in the field frame, the resultant vector points at the real HUB.
     *
     * noteExitSpeedMps is a single averaged speed used only for this angle correction, not
     * ShooterCalculation's per-distance RPS table -- see Variables.NOTE_EXIT_SPEED_MPS for why
     * those two numbers are deliberately not the same thing. Standing still (fieldRelativeVelocityMPS
     * near zero) this reduces to the same heading as the plain overload above.
     *
     * @param robotPose current field pose
     * @param fieldRelativeVelocityMPS current FIELD-relative (vx, vy) chassis velocity, not
     *        robot-relative -- see CommandSwerveDrivetrain's getState().Speeds doc, that's
     *        robot-relative and needs rotating by robotPose.getRotation() first.
     * @param noteExitSpeedMps averaged note exit speed, see Variables.NOTE_EXIT_SPEED_MPS
     */
    public static Rotation2d computeTargetDirection(
            Pose2d robotPose, Translation2d fieldRelativeVelocityMPS, double noteExitSpeedMps) {
        Translation2d hub = Constants.getTeamHubTranslation();
        Translation2d toHub = hub.minus(robotPose.getTranslation());
        double distanceMeters = toHub.getNorm();

        if (distanceMeters < 1e-3) {
            // Sitting (almost) on top of the HUB -- direction is meaningless either way, and
            // dividing by ~zero distance below would blow up. Fall back to the uncompensated
            // (equally meaningless) direction instead of NaN-ing out.
            return Rotation2d.fromRadians(Math.atan2(toHub.getY(), toHub.getX()));
        }

        // The field-frame velocity the shot needs once it leaves the robot, pointed straight
        // at the HUB at noteExitSpeedMps...
        Translation2d desiredFieldVelocity = toHub.times(noteExitSpeedMps / distanceMeters);
        // ...minus the velocity the robot is already contributing, leaves the velocity (and so
        // the direction) the shooter itself still needs to add.
        Translation2d requiredRelativeShotVelocity = desiredFieldVelocity.minus(fieldRelativeVelocityMPS);

        return Rotation2d.fromRadians(
                Math.atan2(requiredRelativeShotVelocity.getY(), requiredRelativeShotVelocity.getX()));
    }

    @Override
    public void initialize() {
        cameraSubsystem.setHubPrecisionMode(true);
        hasReseeded = false;
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
        // using the flat HUB_ALIGN_POS_STD_DEV_M std dev (rotation is always
        // LimelightConstants.VISION_ROTATION_STD_DEV, regardless of mode) while hub precision
        // mode is on (set above in initialize()). This replaces reading
        // cameraSubsystem.getCameraOnlyPose() directly, which also removes the old sim-only
        // fallback branch -- getState().Pose is always populated, in sim and on the real robot
        // alike, so there's no more "no camera pose this cycle" case to handle.
        Pose2d robotPose = swerveDrive.getState().Pose;

        // getState().Speeds is robot-relative (see CommandSwerveDrivetrain) -- rotate into the
        // field frame so it lines up with the HUB translation for the lead-compensation math.
        ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                swerveDrive.getState().Speeds, robotPose.getRotation());
        Translation2d fieldRelativeVelocityMPS = new Translation2d(
                fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond);

        Rotation2d targetDirection = computeTargetDirection(
                robotPose, fieldRelativeVelocityMPS, easeOfLife.getNoteExitSpeedMps());

        // Autonomous alignment reseed: once we're both close to on-target and the vision pose
        // is stable, hard-reset odometry to the vision estimate -- once, ever, per command
        // instance (hasReseeded). Deliberately does not run in teleop: a sudden pose snap
        // mid-drive isn't something a driver should have to account for, this is specifically
        // an autonomous-routine confidence boost. See Limelight.checkAutonomousReseed's doc
        // for exactly what "reseed" means here (a resetPose(), not a soft vision nudge).
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