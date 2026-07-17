package frc.robot.commands.AutoAlign;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Variables;
import frc.robot.constants.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.EaseofLife;
import frc.robot.util.NetworkTables;
import frc.robot.vision.Limelight;

import edu.wpi.first.math.util.Units;
public class AlignToHub extends Command {

    private final SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Limelight CameraSubsystem;
    private final CommandSwerveDrivetrain swerveDrive;
    EaseofLife EaseofLife;
    private final ProfiledPIDController rotationPID;
    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier leftSupplier;
    private boolean hasValidTarget = false;

    /**
     * Rotates the robot to face the Hub while allowing the driver
     * to still control forward/lateral movement.
     *
     * Rotation is driven entirely by the Limelight's camera-only pose. If a
     * fresh camera pose isn't available on a given cycle, no new rotation
     * command is issued — the robot holds its current heading until the
     * camera picks the tag back up.
     *
     * @param swerveDrive     the drivetrain subsystem
     * @param CameraSubsystem the vision subsystem
     * @param forwardSupplier field-centric percent max speed (forward)
     * @param leftSupplier    field-centric percent max speed (left)
     */

    public AlignToHub(
        Limelight CameraSubsystem,
        EaseofLife EaseOfLife,
        CommandSwerveDrivetrain swerveDrivetrain,
        DoubleSupplier forwardSupplier,
        DoubleSupplier leftSupplier) {
        
        this.swerveDrive = swerveDrivetrain;
        this.CameraSubsystem = CameraSubsystem;
        this.EaseofLife = EaseOfLife;
        this.forwardSupplier = forwardSupplier;
        this.leftSupplier = leftSupplier;

        rotationPID = new ProfiledPIDController(
            Variables.AlignToHubP,
            Variables.AlignToHubI,
            Variables.AlignToHubD,
            new TrapezoidProfile.Constraints(Math.PI / 2, Math.PI));
        rotationPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        // No reset here — we may not have a camera pose yet at button-press.
        // The PID seeds itself off the first valid camera pose in execute().
        hasValidTarget = false;
    }

    @Override
    public void execute() {
        double velocityX = forwardSupplier.getAsDouble();
        double velocityY = leftSupplier.getAsDouble();
        double rotationalRate = 0;

        rotationPID.setPID(
            EaseofLife.getAlignP(),
            EaseofLife.getAlignI(),
            EaseofLife.getAlignD()
        );

        Optional<Pose2d> cameraPose = CameraSubsystem.getCameraOnlyPose();

        if (cameraPose.isPresent()) {
            Pose2d robotPose = cameraPose.get();

            if (!hasValidTarget) {
                rotationPID.reset(robotPose.getRotation().getRadians());
                hasValidTarget = true;
            }

            Translation2d hubTarget = Constants.getTeamHubTranslation();
            Translation2d toHub = hubTarget.minus(robotPose.getTranslation());
            double targetAngle = Math.atan2(toHub.getY(), toHub.getX());

            NetworkTables.putTargetAngle(Units.radiansToDegrees(targetAngle));
            rotationPID.setGoal(targetAngle);

            rotationalRate = rotationPID.calculate(
                robotPose.getRotation().getRadians()
            ) * Variables.MaxAngularRate;
        }
        // else: no fresh camera pose this cycle rotationalRate stays 0,
        // holding the current heading instead of rotating off a guess.

        swerveDrive.setControl(
            request
                .withDeadband(Variables.getMaxSpeed()* 0.1)
                .withVelocityX(velocityX)
                .withVelocityY(velocityY)
                .withRotationalRate(rotationalRate));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}