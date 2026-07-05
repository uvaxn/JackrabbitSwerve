package frc.robot.subsystems.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Vars;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.DriveInputs;
import frc.robot.subsystems.EaseofLife;
import frc.robot.util.nt;

public class IntakeSubsystem extends SubsystemBase {

    private final TalonFX intakeMotor;
    private final TalonFX dropMotor;
    private final DigitalInput upperSensor;
    private final DigitalInput lowerSensor;
    EaseofLife MotorMode;
    private final DriveInputs driveInputs;
    private double DROP_SPEED      = 0.15;
    private double LIFT_SPEED      = 0.1;
    private static final double SOFT_LIMIT_DOWN = 50.0;
    private static final double SOFT_LIMIT_UP   = 2.0;

    private static final double INTAKE_COLLECT_SPEED = -0.6; // collecting from ground
    private static final double INTAKE_FEED_SPEED    =  -0.2; // feeding up into shooter

    private final CoastOut    coastOut    = new CoastOut();
    private final StaticBrake staticBrake = new StaticBrake();

    private enum DropState { IDLE, MOVING_DOWN, MOVING_UP }

    private DropState state = DropState.IDLE;
    // what "Seeded" means in these variables is basically has it set off the top sensor (it stores that) 
    private boolean hasSeededTop    = true; // the nail is somewhat bent at the top, so it wont set off the top sensor too well. Best to leave this true.
    // same thing except for it's the bottom sensor
    private boolean hasSeededBottom = false;

    public IntakeSubsystem(TalonFX intakeMotor, TalonFX dropMotor,
                        DigitalInput upperSensor, DigitalInput lowerSensor, EaseofLife easeOfLife, DriveInputs DriveInputs) {
        this.intakeMotor = intakeMotor;
        this.dropMotor   = dropMotor;
        this.upperSensor = upperSensor;
        this.lowerSensor = lowerSensor;
        this.MotorMode = easeOfLife;     
        this.driveInputs = DriveInputs;
        dropMotor.setPosition(0.0);
        dropMotor.setControl(staticBrake);

    }
    public void requestDown() {
        if (isAtBottom() && !edu.wpi.first.wpilibj.RobotBase.isSimulation()) return;
        state = DropState.MOVING_DOWN;
        nt.putRobotState("going down");
    }
    public void requestUp() {
        if (isAtTop() && !edu.wpi.first.wpilibj.RobotBase.isSimulation()) return;
        state = DropState.MOVING_UP;
        nt.putRobotState("going up");
    }
    public void startIntake() {
        MotorMode.setSpeed(intakeMotor, INTAKE_COLLECT_SPEED);
        nt.putRobotState("intaking");
        Vars.MaxSpeed = 0.3 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    CommandScheduler.getInstance().schedule(driveInputs.rumblePulse(1, 0.5, 0.1, 0.2));
        
    }
    public void stopIntake() {
        nt.putRobotState("stop intaking");
        MotorMode.setSpeed(intakeMotor, 0);
        Vars.MaxSpeed = 0.7 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    }
    public void startFeeding() {
        MotorMode.setSpeed(intakeMotor, INTAKE_FEED_SPEED);
        if (isAtTop() && !edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
            dropMotor.setControl(staticBrake);
            state = DropState.IDLE;
            return;
        }
        state = DropState.MOVING_UP;
        nt.putRobotState("going up");
    }

    public void stopFeeding() {
        intakeMotor.stopMotor();
        dropMotor.setControl(staticBrake);
        state = DropState.IDLE;
        nt.putRobotState("idle");
    }

    /** @return true when the arm is not moving useful for command isFinished() checks */
    public boolean isIdle() {
        return state == DropState.IDLE;
    }

    /** @return true when arm is fully down and intake is collecting */
    public boolean isCollecting() {
        return state == DropState.IDLE && isAtBottom();
    }

    /** @return true when arm is fully up */
    public boolean isFullyUp() {
        return state == DropState.IDLE && isAtTop();
    }

    public boolean isAtBottom() { return !lowerSensor.get(); }
    public boolean isAtTop()    { return !upperSensor.get(); }

    @Override
    public void periodic() {
        double pos = dropMotor.getPosition().getValueAsDouble();

        if (isAtTop() && !hasSeededTop) {
            dropMotor.setPosition(0.0);
            hasSeededTop    = true;
            hasSeededBottom = false;
            
        }
        if (isAtBottom() && !hasSeededBottom) {
            dropMotor.setPosition(SOFT_LIMIT_DOWN);
            hasSeededBottom = true;
            hasSeededTop    = false;
        }
        
        switch (state) {
            case MOVING_DOWN -> {
                if (isAtBottom() || pos >= SOFT_LIMIT_DOWN) {
                    dropMotor.setControl(coastOut);
                    state = DropState.IDLE;
                    
                } else {
                    dropMotor.set(-DROP_SPEED);
                }
            }
            case MOVING_UP -> {
                if (isAtTop() || pos <= SOFT_LIMIT_UP) {
                    dropMotor.setControl(staticBrake);
                    state = DropState.IDLE;
                    
                } else {
                    dropMotor.set(LIFT_SPEED);
                }
            }
            case IDLE -> {}
        }
    }
}