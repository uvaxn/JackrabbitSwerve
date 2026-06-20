package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.vision.LimelightHelpers;

public class Robot extends TimedRobot {

    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;

    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
        m_robotContainer.easeOfLife.teleopInit();
        LimelightHelpers.SetIMUMode("limelight", 4); // use the ll4 imu, and the pigeon gyro
    }
    @Override public void disabledInit() {
        LimelightHelpers.SetIMUMode("limelight", 1); // re-seed 
    }
    @Override public void disabledPeriodic() {}
    @Override public void disabledExit() {}
    @Override public void autonomousPeriodic() {}
    @Override public void autonomousExit() {}
    @Override public void teleopPeriodic() {}
    @Override public void teleopExit() {}
    @Override public void testInit() { CommandScheduler.getInstance().cancelAll(); }
    @Override public void testPeriodic() {}
    @Override public void testExit() {}
    @Override public void simulationInit() {

    }
    @Override public void simulationPeriodic() {}
    
}