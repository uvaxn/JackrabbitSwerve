package frc.robot.subsystems;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.controls.EaseofLife;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSubsystem extends SubsystemBase {

    private final TalonFX intakeMotor;
    private final TalonFX dropMotor;
    private final DigitalInput upperSensor;
    private final DigitalInput lowerSensor;
    EaseofLife MotorMode = new EaseofLife();

    private static final double DROP_SPEED      = 0.15;
    private static final double LIFT_SPEED      = 0.15;
    private static final double SOFT_LIMIT_DOWN = 50.0;
    private static final double SOFT_LIMIT_UP   = 2.0;

    // TODO: motor speeds tune these
    private static final double INTAKE_COLLECT_SPEED = -0.6; // collecting from ground
    private static final double INTAKE_FEED_SPEED    =  0.6; // feeding up into shooter

    private final CoastOut    coastOut    = new CoastOut();
    private final StaticBrake staticBrake = new StaticBrake();

    private enum DropState { IDLE, MOVING_DOWN, MOVING_UP }
    private DropState state = DropState.IDLE;

    private boolean hasSeededTop    = false;
    private boolean hasSeededBottom = false;

    public IntakeSubsystem(TalonFX intakeMotor, TalonFX dropMotor,
                           DigitalInput upperSensor, DigitalInput lowerSensor) {
        this.intakeMotor = intakeMotor;
        this.dropMotor   = dropMotor;
        this.upperSensor = upperSensor;
        this.lowerSensor = lowerSensor;

        dropMotor.setPosition(0.0);
        dropMotor.setControl(staticBrake);

        // Drop intake on init so it's ready to collect immediately
        requestDown();
    }
    /** Called automatically on init. Drops arm to collecting position. */
    public void requestDown() {
        if (isAtBottom() && !edu.wpi.first.wpilibj.RobotBase.isSimulation()) return;
        state = DropState.MOVING_DOWN;
        MotorMode.setAutoState("going down");
    }
    public void startIntake() {
        MotorMode.setSpeed(dropMotor, INTAKE_COLLECT_SPEED);
    }
    public void stopIntake() {
        MotorMode.setSpeed(dropMotor, 0);
    }
    public void startFeeding() {
        intakeMotor.set(INTAKE_FEED_SPEED); // feed balls toward shooter
        if (isAtTop() && !edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
            dropMotor.setControl(staticBrake);
            state = DropState.IDLE;
            return;
        }
        state = DropState.MOVING_UP;
        MotorMode.setAutoState("going up");
    }

    public void stopFeeding() {
        intakeMotor.stopMotor();
        dropMotor.setControl(staticBrake);
        state = DropState.IDLE;
        MotorMode.setAutoState("idle");
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
                    intakeMotor.set(INTAKE_COLLECT_SPEED); // start collecting at bottom
                    state = DropState.IDLE;
                    System.out.println("Intake: reached bottom, now collecting.");
                } else {
                    dropMotor.set(-DROP_SPEED);
                }
            }
            case MOVING_UP -> {
                if (isAtTop() || pos <= SOFT_LIMIT_UP) {
                    dropMotor.setControl(staticBrake);
                    // NOTE: intake motor keeps spinning (set by startFeeding) until stopFeeding() is called
                    state = DropState.IDLE;
                    System.out.println("Intake: reached top.");
                } else {
                    dropMotor.set(LIFT_SPEED);
                }
            }
            case IDLE -> {}
        }

        SmartDashboard.putNumber("IntakeDrop/Rotation",  dropMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("IntakeDrop/Velocity",  dropMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putString("IntakeDrop/State",     state.toString());
        SmartDashboard.putBoolean("IntakeDrop/AtTop",    isAtTop());
        SmartDashboard.putBoolean("IntakeDrop/AtBottom", isAtBottom());
        SmartDashboard.putBoolean("IntakeDrop/IsIdle",   isIdle());
    }
}