package frc.robot;

// wayy cleaner than the yucky old robot container

/*
 * CONTROLS 
 * 
 * Left Joystick -- Moves robot according to joystick
 * 
 * Right Joystick -- Rotates robot according to joystick 
 * 
 * Left Trigger -- Intake (Drops down, and intakes balls.)
 * 
 * Right Trigger -- Shoot out balls 
 * 
 * Left Bumper -- Resets where position is according to field (when pressed; if its facing toward you, then that is it's forward now.)
 * 
 * X button -- Face toward hub
 * 
 * Y button -- no clue but does something to wheels
 * 
 * Remember to move the intake drop, so that it is up when we start the robot.
 */
import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AutoAlign;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.controls.Shooters;
public class RobotContainer {
    private double MaxSpeed = 0.6 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(Vars.MaxAngularRate).in(RadiansPerSecond);
    
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.03).withRotationalDeadband(MaxAngularRate * 0.03)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController joystick = new CommandXboxController(0);
    private SendableChooser<Command> autoChooser;
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final CameraSubsystem cameraSubsystem = new CameraSubsystem(drivetrain);
    
    public final AutoAlign AligntoHub = new AutoAlign(drivetrain, cameraSubsystem, () -> -joystick.getLeftY(),  () -> -joystick.getLeftX(),  0.5);
    double matchTime = DriverStation.getMatchTime();
    // Motors
    public final TalonFX m_ShooterR    = new TalonFX(23);
    public final TalonFX m_ShooterL    = new TalonFX(22);
    public final TalonFX m_Intake      = new TalonFX(25);
    public final TalonFX m_IntakeDrop  = new TalonFX(20);
    public final TalonFX m_LowerFeed   = new TalonFX(21);
    public final TalonFX m_UpperFeed   = new TalonFX(24);


    public final Shooters shooters = new Shooters(m_ShooterR, m_ShooterL, m_LowerFeed, m_UpperFeed);

    public final IntakeSubsystem intakes = new IntakeSubsystem(
        m_Intake, m_IntakeDrop,
        new DigitalInput(0),
        new DigitalInput(1)
    );
    
    public RobotContainer() {
        NamedCommands.registerCommand("shoot", new InstantCommand(() -> shooters.shoot()));
        NamedCommands.registerCommand("stop shoot", new InstantCommand(() -> shooters.stopShoot()));
        NamedCommands.registerCommand("intake", new InstantCommand(intakes::requestDown, intakes));
        NamedCommands.registerCommand("intake up", new InstantCommand(intakes::requestUp, intakes));
        configureBindings();
        drivetrain.configureAutoBuilder();

        autoChooser = AutoBuilder.buildAutoChooser("None");
        SmartDashboard.putData("Auto", autoChooser);
        
    }
    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double controller_x = joystick.getLeftY();
                double controller_y = joystick.getLeftX();
                double controller_rot = -joystick.getRightX();
                controller_x = Math.copySign(controller_x * controller_x, controller_x);
                controller_y = Math.copySign(controller_y * controller_y, controller_y);
                return drive
                    .withVelocityX(
                        Vars.xLimiter.calculate(
                            controller_x * MaxSpeed
                        )
                    )
                    .withVelocityY(
                        Vars.yLimiter.calculate(
                            controller_y * MaxSpeed
                        )
                    )
                    .withRotationalRate(
                        controller_rot * MaxAngularRate
                    );
            })
        );
        
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.y().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // SysId routines
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset field centric heading
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        joystick.x().whileTrue(AligntoHub);
        // Intake Mechanisms

        joystick.leftTrigger()
            .onTrue(new InstantCommand(intakes::requestDown, intakes))
            .onFalse(new InstantCommand(intakes::requestUp,  intakes));
        // Shooting Mechanisms
        joystick.rightTrigger()
            .onTrue(new InstantCommand(() -> shooters.shoot()));
        joystick.rightTrigger()
            .onFalse(new InstantCommand(() -> shooters.stopShoot())); 
        drivetrain.registerTelemetry(logger::telemeterize); 
        }
        
    public Command getAutonomousCommand() {
        Command selected = autoChooser.getSelected();
       
        if (selected == null) return Commands.none();
        System.out.println("auto: " + autoChooser.getSelected().getName());
        return selected;
    }
}
