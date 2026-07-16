package frc.robot.commands.AutoAlign;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vars;
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

    /**
     * Rotates the robot to face the Hub while allowing the driver
     * to still control forward/lateral movement.
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
            Vars.AlignToHubP,
            Vars.AlignToHubI,
            Vars.AlignToHubD,
            new TrapezoidProfile.Constraints(Math.PI / 2, Math.PI));
        rotationPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        Pose2d startPose = CameraSubsystem.getEstimatedPose()
            .orElse(swerveDrive.getState().Pose);
        rotationPID.reset(startPose.getRotation().getRadians());
    }

    @Override
    public void execute() {
        // falls back to odometry if no cam pose is present 
        Pose2d robotPose = CameraSubsystem.getEstimatedPose()
            .orElse(swerveDrive.getState().Pose);

        double velocityX = forwardSupplier.getAsDouble();
        double velocityY = leftSupplier.getAsDouble();
        double rotationalRate = 0;
        

        rotationPID.setPID(
            EaseofLife.getAlignP(),
            EaseofLife.getAlignI(),
            EaseofLife.getAlignD()
        );
        // Pick hub based on alliance
        Translation2d hubTarget = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
            ? Constants.redHubPosition
            : Constants.blueHubPosition;
        // Publish angles
        Translation2d RobotPos = robotPose.getTranslation();

        Translation2d toHub = hubTarget.minus(RobotPos);

        double targetAngle = Math.atan2(toHub.getY(), toHub.getX());

        NetworkTables.putTargetAngle(Units.radiansToDegrees(targetAngle));

        rotationPID.setGoal(targetAngle);

        rotationalRate = rotationPID.calculate(
            robotPose.getRotation().getRadians()
        ) * Vars.MaxAngularRate;

        swerveDrive.setControl(
            request
                .withDeadband(Vars.MaxSpeed * 0.1)
                .withVelocityX(velocityX)
                .withVelocityY(velocityY)
                .withRotationalRate(rotationalRate));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}