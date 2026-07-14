package frc.robot.commands.AutoAlign;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vars;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.EaseofLife;
import frc.robot.util.nt;

public class AlignToAllianceWall extends Command {

    private final SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric()
        .withDeadband(Vars.MaxSpeed * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final CommandSwerveDrivetrain swerveDrive;
    private final ProfiledPIDController rotationPID;
    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier leftSupplier;

    /**
     * Rotates the robot to face its alliance wall (0° for Blue, π for Red)
     * while allowing the driver to still control forward/lateral movement.
     *
     * @param swerveDrive     the drivetrain subsystem
     * @param easeOfLife      tuning helper for live PID adjustment
     * @param forwardSupplier field-centric percent max speed (forward)
     * @param leftSupplier    field-centric percent max speed (left)
     */
    public AlignToAllianceWall(
        CommandSwerveDrivetrain swerveDrive,
        EaseofLife easeOfLife,
        DoubleSupplier forwardSupplier,
        DoubleSupplier leftSupplier) {

        this.swerveDrive = swerveDrive;
        this.forwardSupplier = forwardSupplier;
        this.leftSupplier = leftSupplier;

        rotationPID = new ProfiledPIDController(
            Vars.AlignToAllianceWallP,
            Vars.AlignToAllianceWallI,
            Vars.AlignToAllianceWallD,
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
        if (possiblePose.isEmpty()) return;

        Pose2d robotPose = possiblePose.get();

        rotationPID.setPID(
            Vars.AlignToAllianceWallP,
            Vars.AlignToAllianceWallI,
            Vars.AlignToAllianceWallD
        );

        // Blue faces 0° (positive X), Red faces 180° (negative X / their wall) Measured in Radians 
        double targetAngle = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
            ? 0
            : Math.PI;

        nt.putTargetAngle(Units.radiansToDegrees(targetAngle));

        rotationPID.setGoal(targetAngle);

        double rotationalRate = rotationPID.calculate(
            robotPose.getRotation().getRadians()
        ) * Vars.MaxAngularRate;

        swerveDrive.setControl(
            request
                .withVelocityX(forwardSupplier.getAsDouble())
                .withVelocityY(leftSupplier.getAsDouble())
                .withRotationalRate(rotationalRate));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}