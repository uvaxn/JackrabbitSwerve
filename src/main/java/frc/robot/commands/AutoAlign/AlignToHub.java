package frc.robot.commands.AutoAlign;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vars;
import frc.robot.Constants;
import frc.robot.subsystems.EaseofLife;
import frc.robot.subsystems.robot.CommandSwerveDrivetrain;
import frc.robot.util.nt;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
public class AlignToHub extends Command {

    private final SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric()
        .withDeadband(Vars.MaxSpeed * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final CommandSwerveDrivetrain swerveDrive;
    
    EaseofLife easeofLife;
    private final ProfiledPIDController rotationPID;
    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier leftSupplier;

    /**
     * Rotates the robot to face the nearest AprilTag while allowing the driver
     * to still control forward/lateral movement.
     *
     * @param swerveDrive     the drivetrain subsystem
     * @param cameraSubsystem the vision subsystem
     * @param forwardSupplier field-centric percent max speed (forward)
     * @param leftSupplier    field-centric percent max speed (left)
     */

    public AlignToHub(
        CommandSwerveDrivetrain swerveDrive,
        EaseofLife easeOfLife,
        DoubleSupplier forwardSupplier,
        DoubleSupplier leftSupplier) {
            
        this.swerveDrive = swerveDrive;
        this.easeofLife = easeOfLife;
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
        Translation2d robotVelocity = new Translation2d(
            swerveDrive.getState().Speeds.vxMetersPerSecond,
            swerveDrive.getState().Speeds.vyMetersPerSecond
        );
        double velocityX = forwardSupplier.getAsDouble();
        double velocityY = leftSupplier.getAsDouble();
        double rotationalRate = 0;
        
        if (possiblePose.isEmpty()) return;
        Pose2d robotPose = possiblePose.get();
        rotationPID.setPID(
            // more P and D, minimize I as much as you can
            easeofLife.getAlignP(),
            easeofLife.getAlignI(),
            easeofLife.getAlignD()
            //  TODO: -------------->>>>>>>>>>>>>> adjust PID for AutoAlign!!!!! <<<<<<<<<<<<--------------------
        );
        // Pick hub based on alliance
        Translation2d hubTarget = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
            ? Constants.redHubPosition
            : Constants.blueHubPosition;

        Translation2d toHub = hubTarget.minus(robotPose.getTranslation());

        double dist = toHub.getNorm();

        double flightTime = dist / Vars.airTimeScalarSeconds;
        // Publish angles
        Translation2d predictedRobotPos = robotPose.getTranslation()
            .plus(robotVelocity.times(flightTime));

        Translation2d toHubPredicted = hubTarget.minus(predictedRobotPos);

        double targetAngle = Math.atan2(toHubPredicted.getY(), toHubPredicted.getX());

        nt.putTargetAngle(Units.radiansToDegrees(targetAngle));
        SmartDashboard.putNumberArray("Pose/predictedRobotPos", new double[]{
            predictedRobotPos.getX(),
            predictedRobotPos.getY(),
            robotPose.getRotation().getDegrees()
        });
        SmartDashboard.putNumberArray("Pose/hubTarget", new double[]{
            hubTarget.getX(),
            hubTarget.getY(),
            0.0
        });
        SmartDashboard.putNumber(
            "AutoAlign/RobotAngleDeg",
            robotPose.getRotation().getDegrees()
        );

        rotationPID.setGoal(targetAngle);

        rotationalRate = rotationPID.calculate(
            robotPose.getRotation().getRadians()
        ) * Vars.MaxAngularRate;

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