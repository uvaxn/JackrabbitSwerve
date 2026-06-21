package frc.robot;

/*
 * CONTROLS 
 * 
 * Left Joystick  -- Moves robot
 * Right Joystick -- Rotates robot
 * Left Trigger   -- Intake (drops down and intakes)
 * Right Trigger  -- Shoot
 * Left Bumper    -- Reset field-centric heading
 * X Button       -- Auto-align to nearest AprilTag
 * Y Button       -- Auto-align to alliance Wall
 * A Button       -- Points wheels based on joystick direction
 */

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AutoAlign.AlignToAllianceWall;
import frc.robot.commands.AutoAlign.AlignToHub;
import frc.robot.controls.EaseofLife;
import frc.robot.controls.Shooters;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.vision.CameraSubsystem;

public class RobotContainer {

    private final double MaxAngularRate = RotationsPerSecond.of(Vars.MaxAngularRate).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(Vars.MaxSpeed * 0.01)
        .withRotationalDeadband(MaxAngularRate * 0.003)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Telemetry logger = new Telemetry(Vars.MaxSpeed);
    private final CommandXboxController joystick = new CommandXboxController(0);

    private SendableChooser<Command> autoChooser;
    
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final CameraSubsystem cameraSubsystem = new CameraSubsystem(drivetrain);
    public final EaseofLife easeOfLife = new EaseofLife(cameraSubsystem);
    // Motors
    public final TalonFX m_ShooterR   = new TalonFX(23);
    public final TalonFX m_ShooterL   = new TalonFX(22);
    public final TalonFX m_Intake     = new TalonFX(25);
    public final TalonFX m_IntakeDrop = new TalonFX(20);
    public final TalonFX m_LowerFeed  = new TalonFX(21);
    public final TalonFX m_UpperFeed  = new TalonFX(24);

    
    
    public final IntakeSubsystem intakes = new IntakeSubsystem(
        m_Intake, m_IntakeDrop,
        new DigitalInput(0),
        new DigitalInput(1),
        easeOfLife
    );
    public final Shooters shooters = new Shooters(m_ShooterR, m_ShooterL, m_LowerFeed, m_UpperFeed, easeOfLife, cameraSubsystem, intakes);
    // Commands
    public RobotContainer() {
        NamedCommands.registerCommand("shoot",      new InstantCommand(() -> shooters.shoot()));
        NamedCommands.registerCommand("stop shoot", new InstantCommand(() -> shooters.stopShoot()));
        NamedCommands.registerCommand("intake",     new InstantCommand(intakes::startIntake, intakes));
        NamedCommands.registerCommand("intake up",  new InstantCommand(intakes::stopIntake,  intakes));

        drivetrain.configureAutoBuilder();
        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser("None");
        SmartDashboard.putData("Auto", autoChooser);
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double limitedX = Vars.xLimiter.calculate(joystick.getLeftY()  * Vars.MaxSpeed);
                double limitedY = Vars.yLimiter.calculate(joystick.getLeftX()  * Vars.MaxSpeed);
                Vars.lastLimitedX = limitedX;
                Vars.lastLimitedY = limitedY;
                return drive
                    .withVelocityX(limitedX)
                    .withVelocityY(limitedY)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate);
            })
        );

        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(SwerveRequest.Idle::new).ignoringDisable(true)
        );

        // Wheel point mode
        joystick.a().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // SysId routines
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset field-centric heading
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Auto-align to nearest AprilTag
        joystick.x()
        .whileTrue(new AlignToHub(
            drivetrain,
            easeOfLife,
            () -> Vars.xLimiter.calculate(joystick.getLeftY() * Vars.MaxSpeed),
            () -> Vars.yLimiter.calculate(joystick.getLeftX() * Vars.MaxSpeed)
        ));
        // align to alliance wall is the exact same thing btw, just toward your alliance wall ^
        joystick.y()
        .whileTrue(new AlignToAllianceWall(
            drivetrain, 
            easeOfLife,             
            () -> Vars.xLimiter.calculate(joystick.getLeftY() * Vars.MaxSpeed),
            () -> Vars.yLimiter.calculate(joystick.getLeftX() * Vars.MaxSpeed)
        ));
        // Intake
        joystick.leftTrigger()
            .onTrue(new InstantCommand(intakes::startIntake, intakes))
            .onFalse(new InstantCommand(intakes::stopIntake,  intakes));

        // Shooter
        joystick.rightTrigger()
            .onTrue(new InstantCommand(() -> shooters.shoot()))
            .onFalse(new InstantCommand(() -> shooters.stopShoot()));
        joystick.pov(180)
            .onTrue(new InstantCommand(() -> shooters.simpleShoot()))
            .onFalse(new InstantCommand(() -> shooters.stopShoot()));
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        Command selected = autoChooser.getSelected();
        if (selected == null) return Commands.none();
        System.out.println("auto: " + selected.getName());
        return selected;
    }
}