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
        // .withDeadband(Vars.MaxSpeed * 0.03) -- this has 3 percent deadband
        .withRotationalDeadband(MaxAngularRate * 0.03) // was 0.03 (3%)
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

        // Reset field centric heading, odometry points toward alliance wall.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Y toggles auto-align mode on/off (on by default -- see EaseofLife.autoAlignEnabled).
        // Published live to NetworkTables as Info/AlignMode.
        joystick.y().onTrue(new InstantCommand(easeOfLife::toggleAutoAlign));

        // While a firing sequence is spinning up or actively firing (either feed mode, see
        // Mechanisms.isShooting()) and auto-align is enabled: face the HUB when we're in our
        // own alliance zone, or our own alliance wall otherwise -- even from inside the
        // opposing alliance's zone. Re-checked every cycle, see AlignWhileShooting.
        //
        // Teleop-gated on purpose: this Trigger schedules AlignWhileShooting independently of
        // whatever's currently running, which requires drivetrain. A PathPlanner auto's whole
        // SequentialCommandGroup holds drivetrain as a requirement for its ENTIRE scheduled
        // lifetime (the union of every sub-command's requirements, not just whichever step is
        // currently active) -- so if this fired during autonomous, the moment "shoot" made
        // isShooting() true, this would forcibly cancel the ENTIRE auto, not just hand off
        // cleanly, because the scheduler sees two competing claims on drivetrain. See the
        // "shootWithAlign"-style NamedCommand note in AlignWhileShooting's class doc for how
        // to get this same behavior safely during auto (composed inside the auto's own command
        // tree instead of racing it from outside).
        new Trigger(() -> DriverStation.isTeleopEnabled()
                && mechanisms.isShooting()
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

        // Shooter spins for as long as right trigger is held, stops on release. Regular-mode
        // feed (the right-bumper distinction) has been removed -- there's only one shooting
        // behavior now, so this is just a plain onTrue/onFalse pair, no compound triggers.
        joystick.rightTrigger()
            .onTrue(new InstantCommand(mechanisms::startShooting, mechanisms))
            .onFalse(new InstantCommand(mechanisms::stopShooting, mechanisms));
        joystick.povDown()
            .onTrue(new InstantCommand(intakeDrop::requestDown, intakeDrop));
        joystick.povUp()
            .onTrue(new InstantCommand(intakeDrop::requestUp, intakeDrop));
        drivetrain.registerTelemetry(logger::telemeterize);
        
    }

    public Command getAutonomousCommand() {
        Command selected = autoChooser.getSelected();
        if (selected == null) return Commands.none();
        System.out.println("Auto: " + selected.getName());
        return selected;
    }
}