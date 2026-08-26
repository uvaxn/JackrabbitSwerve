package frc.robot;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.MathUtil;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AutoAlign.AlignToAllianceWall;
import frc.robot.commands.AutoAlign.AlignWhileShooting;
import frc.robot.constants.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DriveInputs;
import frc.robot.subsystems.EaseofLife;
import frc.robot.subsystems.Mechanisms;
import frc.robot.subsystems.robot.FeedSubsystem;
import frc.robot.subsystems.robot.IntakeDropSubsystem;
import frc.robot.subsystems.robot.IntakeSubsystem;
import frc.robot.subsystems.robot.ShooterSubsystem;
import frc.robot.vision.Limelight;

public class RobotContainer {

    private final double MaxAngularRate = RotationsPerSecond.of(Variables.MaxAngularRate).in(RadiansPerSecond);
    
    private static final double ROTATION_STICK_DEADBAND = 0.08; // matches DriveInputs' X/Y deadband

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withRotationalDeadband(MaxAngularRate * 0.03) 
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Telemetry logger = new Telemetry(Variables.getMaxSpeed());
    private final CommandXboxController joystick = new CommandXboxController(0);

    public final DriveInputs driveInputs = new DriveInputs(
        () -> joystick.getLeftY(),
        () -> joystick.getLeftX(), 
        joystick
    );
    private SendableChooser<Command> autoChooser;
    
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    public final Limelight cameraSubsystem = new Limelight(Constants.LL_NAME);

    public final EaseofLife easeOfLife = new EaseofLife(cameraSubsystem);

    // Motors
    public final TalonFX m_ShooterR   = new TalonFX(Constants.m_ShooterR);
    public final TalonFX m_ShooterL   = new TalonFX(Constants.m_ShooterL);
    public final TalonFX m_Intake     = new TalonFX(Constants.m_Intake);
    public final TalonFX m_IntakeDrop = new TalonFX(Constants.m_IntakeDrop);
    public final TalonFX m_LowerFeed  = new TalonFX(Constants.m_LowerFeed);
    public final TalonFX m_UpperFeed  = new TalonFX(Constants.m_UpperFeed);
 
    
    
    public final IntakeDropSubsystem intakeDrop = new IntakeDropSubsystem(
        m_IntakeDrop,

        new DigitalInput(0),
        new DigitalInput(1)
    );
    public final IntakeSubsystem intakes = new IntakeSubsystem(
        m_Intake,

        easeOfLife
    );
    public final ShooterSubsystem shooters = new ShooterSubsystem(
        m_ShooterR, 
        m_ShooterL, 

        easeOfLife
    );
    public final FeedSubsystem feeds = new FeedSubsystem(
        intakeDrop, 
        easeOfLife, 

        m_LowerFeed, 
        m_UpperFeed
    );
    public final Mechanisms mechanisms = new Mechanisms(
        shooters, 
        intakes, 
        intakeDrop,
        feeds
    );
    // Commands
    public RobotContainer() {
        NamedCommands.registerCommand("shoot",      new InstantCommand(mechanisms::startShooting));
        NamedCommands.registerCommand("stop shoot", new InstantCommand(mechanisms::stopShooting));
        NamedCommands.registerCommand("intake",     new InstantCommand(intakes::start, intakes));
        NamedCommands.registerCommand("stop intake",  new InstantCommand(intakes::stop,  intakes));
        NamedCommands.registerCommand("requestUp", new InstantCommand(intakeDrop::requestUp));
        NamedCommands.registerCommand("requestDown", new InstantCommand(intakeDrop::requestDown));
        drivetrain.configureAutoBuilder(); 
        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser("None");
        SmartDashboard.putData("Auto", autoChooser);
    }

    private void configureBindings() {

        drivetrain.setDefaultCommand( // reminder that this chunk of code basically controls the movement of the robot
            drivetrain.applyRequest(() ->
                drive.withVelocityX(driveInputs.getX())
                    .withVelocityY(driveInputs.getY())
                    .withRotationalRate(
                        -MathUtil.applyDeadband(joystick.getRightX(), ROTATION_STICK_DEADBAND) * MaxAngularRate)
            )
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

        // Left bumper: fixed-speed backup shot. Bypasses vision/distance calc entirely (see
        // ShooterSubsystem.startFixed() / Variables.FIXED_SHOOTER_SPEED) -- always the same
        // commanded speed regardless of what the Limelight is or isn't seeing right now. Also
        // does NOT trigger auto-align-while-shooting (see Mechanisms.isShootingWithVision() and
        // the Trigger below) -- this is meant to just shoot, nothing else moves.
        joystick.leftBumper()
            .onTrue(new InstantCommand(mechanisms::startShootingFixed, mechanisms))
            .onFalse(new InstantCommand(mechanisms::stopShooting, mechanisms));

        // Y: reset field-centric heading (was the auto-align-while-shooting toggle -- that
        // toggle binding has been removed, see note below).
        joystick.y().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Right bumper: manual "face our alliance wall" while held, independent of shooting --
        // reuses the same AlignToAllianceWall command AlignWhileShooting calls internally when
        // you're out of your own zone, just available on demand instead of only while firing.
        joystick.rightBumper().whileTrue(new AlignToAllianceWall(
            drivetrain,
            easeOfLife,
            driveInputs::getX,
            driveInputs::getY
        ));
        new Trigger(() -> DriverStation.isTeleopEnabled()
                && mechanisms.isShootingWithVision()
                && easeOfLife.isAutoAlignEnabled())
            .whileTrue(new AlignWhileShooting(
                cameraSubsystem,
                easeOfLife,
                drivetrain,
                driveInputs::getX,
                driveInputs::getY
            ));
        // Intake
        joystick.leftTrigger()
            .onTrue(new InstantCommand(mechanisms::requestIntake, mechanisms))
            .onFalse(new InstantCommand(mechanisms::stopIntake, mechanisms));

        // Shooter spins for as long as right trigger is held, stops on release. The old
        // right-bumper feed-mode distinction (full-hopper vs regular) is gone -- right bumper
        // is now the manual face-alliance-wall binding above, unrelated to feed mode -- so
        // this is just a plain onTrue/onFalse pair, no compound triggers.
        joystick.rightTrigger()
            .onTrue(new InstantCommand(mechanisms::startShooting, mechanisms))
            .onFalse(new InstantCommand(mechanisms::stopShooting, mechanisms));
        joystick.povDown()
            .onTrue(new InstantCommand(intakeDrop::requestDown, intakeDrop));
        joystick.povUp()
            .onTrue(new InstantCommand(intakeDrop::requestUp, intakeDrop));

        // D-pad left: shooter + feeds, arm stays put (no bounce). The arm only moves via
        // d-pad up/down above -- this binding never touches IntakeDropSubsystem. Routes
        // through Mechanisms.startShooterAndFeeds(), which runs the feed rollers directly
        // (no arm bounce) and sets a flag so the SPINNING_UP->FIRING auto-feed is skipped.
        joystick.povLeft()
            .onTrue(new InstantCommand(mechanisms::startShooterAndFeeds, mechanisms))
            .onFalse(new InstantCommand(mechanisms::stopShooterAndFeeds, mechanisms));

        // D-pad right: feeds only (no shooter, no arm). Just the upper+lower feed rollers,
        // started/stopped without bouncing or parking the intake-drop arm.
        joystick.povRight()
            .onTrue(new InstantCommand(feeds::rollersStart, feeds))
            .onFalse(new InstantCommand(feeds::rollersStop, feeds));
        drivetrain.registerTelemetry(logger::telemeterize);
        
    }

    public Command getAutonomousCommand() {
        Command selected = autoChooser.getSelected();
        if (selected == null) return Commands.none();
        System.out.println("Auto: " + selected.getName());
        return selected;
    }
}