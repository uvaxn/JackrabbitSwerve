// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import frc.robot.Constants;
import frc.robot.Vars;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CameraSubsystem;
public class AutoAlign extends Command {

  /*
   * Because this command's owner can pass X and Y movements to apply to the robot,
   * it is good to give the owner a fixed frame of reference to talk about.
   * To do this, we make a field centric swerve request.
   * 
   * We apply deadband to the percent max speed supplier from the owner.
   * We do not apply a deadband to our own output to the swerve request.
   * (In the case we would like to deadband out rotation, the PID itself must be deadbanded.)
   * Use open loop voltage because we feed request with numbers from the owner.
   */
  private final SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric()
          .withDeadband(Vars.MaxSpeed * 0.1) // Add a 10% deadband to the caller
          // .withRotationalDeadband(Vars.MaxAngularRate * 0.1) // no rotational deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  // TODO consider using FieldCentricFacingAngle. Then the PID could possibly be baked directly into the swerve request.

  private CommandSwerveDrivetrain swerveDrive;
  private CameraSubsystem cameraSubsystem;
  private ProfiledPIDController rotationPID;
  DoubleSupplier forwardSupplier;
  DoubleSupplier leftSupplier;
  double indexerPercent;

  Translation2d hubTranslation;

  /**
   * Creates a new AlignToHub.
   * 
   * This command rotates the robot to face the center of the alliance hub.
   * The owner of this command must provide an supplier for the forward and left motion of the robot to also get applied to the robot.
   * (These are field centric.)
   * This command also constantly revs the shooter so that shooting can happen immediately when ready.
   * When the robot is pointed within the allowed error of the target and the RPM is within the allowed amount, this command feeds the shooter.
   * 
   * @param commandSwerveDrivetrain the drivetrain subsystem
   * @param shooterSubsystem the shooter subsystem
   * @param indexerSubsystem the indexer subsystem
   * @param forwardSupplier field centric percent max speed supplier
   * @param leftSupplier field centric percent max speed supplier
   * @param indexerPercent the percent to run the indexer at when feeding
   */
    public AutoAlign(CommandSwerveDrivetrain commandSwerveDrivetrain, CameraSubsystem cameraSubsystem, DoubleSupplier forwardSupplier, DoubleSupplier leftSupplier, double indexerPercent) {
        addRequirements(commandSwerveDrivetrain);

        this.swerveDrive = commandSwerveDrivetrain;
        this.cameraSubsystem = cameraSubsystem;
        this.forwardSupplier = forwardSupplier;
        this.leftSupplier = leftSupplier;
        this.indexerPercent = indexerPercent;

        rotationPID = new ProfiledPIDController(Vars.AlignToHubP, Vars.AlignToHubI, Vars.AlignToHubD, new TrapezoidProfile.Constraints(Math.PI / 2, Math.PI));
        rotationPID.enableContinuousInput(-Math.PI, Math.PI);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationPID.reset(swerveDrive.samplePoseAt(Timer.getFPGATimestamp()).get().getRotation().getRadians());

    /*
     * Aquire the hub translation when initializing in case we are testing and change the alliance of our robot.
     */
    hubTranslation = Constants.getTeamHubTranslation();
  }

  // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Optional<Pose2d> possiblePose = swerveDrive.samplePoseAt(Timer.getFPGATimestamp());
        if (possiblePose.isEmpty()) {
            swerveDrive.setControl(
                request.withVelocityX(forwardSupplier.getAsDouble() * Vars.MaxSpeed)
                    .withVelocityY(leftSupplier.getAsDouble() * Vars.MaxSpeed)
                    .withRotationalRate(0)
            );
            return;
        }

        Pose2d robotPose = possiblePose.get();
        Optional<Pose2d> nearestTagPose = cameraSubsystem.getNearestTagPose(robotPose);

        if (nearestTagPose.isPresent()) {
            Pose2d tagPose = nearestTagPose.get();
            Translation2d toTag = tagPose.getTranslation().minus(robotPose.getTranslation());
            double targetAngle = Math.atan2(toTag.getY(), toTag.getX());

            rotationPID.setGoal(targetAngle);
            double output = rotationPID.calculate(robotPose.getRotation().getRadians());

            swerveDrive.setControl(
                request.withVelocityX(forwardSupplier.getAsDouble() * Vars.MaxSpeed)
                    .withVelocityY(leftSupplier.getAsDouble() * Vars.MaxSpeed)
                    .withRotationalRate(output * Vars.MaxAngularRate)
            );
        } else {
            swerveDrive.setControl(
                request.withVelocityX(forwardSupplier.getAsDouble() * Vars.MaxSpeed)
                    .withVelocityY(leftSupplier.getAsDouble() * Vars.MaxSpeed)
                    .withRotationalRate(0)
            );
        }
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    /*
     * When the command ends, it is important to stop all moters.
     * The owner of this command can overwrite this function if this behavior is not desired.
     */
    swerveDrive.setControl(new SwerveRequest.Idle());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
