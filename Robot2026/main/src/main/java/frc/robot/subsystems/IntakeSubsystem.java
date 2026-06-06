package frc.robot.subsystems;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class IntakeSubsystem extends SubsystemBase {

    private final TalonFX intakeMotor;
    private final TalonFX dropMotor;
    private final DigitalInput upperSensor;
    private final DigitalInput lowerSensor;

    // TODO: tune these in motor rotations
    private static final double DROP_SPEED       = 0.15;
    private static final double LIFT_SPEED       = 0.15;
    private static final double SOFT_LIMIT_DOWN  = 50.0; // tune on real robot
    private static final double SOFT_LIMIT_UP    = 2.0;  // small buffer from 0

    private final CoastOut    coastOut    = new CoastOut();
    private final StaticBrake staticBrake = new StaticBrake();

    private enum DropState { IDLE, MOVING_DOWN, MOVING_UP }
    private DropState state = DropState.IDLE;

    public IntakeSubsystem(TalonFX intakeMotor, TalonFX dropMotor,
                           DigitalInput upperSensor, DigitalInput lowerSensor) {
        this.intakeMotor = intakeMotor;
        this.dropMotor   = dropMotor;
        this.upperSensor = upperSensor;
        this.lowerSensor = lowerSensor;

        dropMotor.setPosition(0.0);        // seed encoder assume starting UP
        dropMotor.setControl(staticBrake); // hold arm up on startup
    }

    // Commands

    public void requestDown() {
        if (isAtBottom() && !edu.wpi.first.wpilibj.RobotBase.isSimulation()) return;
        state = DropState.MOVING_DOWN;
        System.out.println("going down");
    }

    public void requestUp() {
        intakeMotor.stopMotor();
        if (isAtTop() && !edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
            dropMotor.setControl(staticBrake);
            state = DropState.IDLE;
            return;
        }
        state = DropState.MOVING_UP;
        System.out.println("going up");
    }

    // Sensors

    public boolean isAtBottom() { return !lowerSensor.get(); }
    public boolean isAtTop()    { return !upperSensor.get(); }
    private boolean hasSeededTop = false;
    private boolean hasSeededBottom = false;

    @Override
    public void periodic() {
        double pos = dropMotor.getPosition().getValueAsDouble();

        // Only reseed once per limit hit, not every tick
        if (isAtTop() && !hasSeededTop) {
            dropMotor.setPosition(0.0);
            hasSeededTop = true;
            hasSeededBottom = false;
        }
        if (isAtBottom() && !hasSeededBottom) {
            dropMotor.setPosition(SOFT_LIMIT_DOWN);
            hasSeededBottom = true;
            hasSeededTop = false;
        }

        switch (state) {
            case MOVING_DOWN -> {
                if (isAtBottom() || pos >= SOFT_LIMIT_DOWN) {
                    dropMotor.setControl(coastOut);
                    intakeMotor.set(0.8); // start intaking when arm reaches bottom
                    state = DropState.IDLE;
                    System.out.println("reached bottom.");
                } else {
                    dropMotor.set(-DROP_SPEED);
                }
            }
            case MOVING_UP -> {
                if (isAtTop() || pos <= SOFT_LIMIT_UP) {
                    dropMotor.setControl(staticBrake);
                    state = DropState.IDLE;
                    System.out.println("reached top.");
                } else {
                    dropMotor.set(LIFT_SPEED);
                }
            }
            case IDLE -> {}
        }

        SmartDashboard.putNumber("IntakeDrop/Rotation", dropMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("IntakeDrop/Velocity", dropMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putString("IntakeDrop/State", state.toString());
        SmartDashboard.putBoolean("IntakeDrop/AtTop", isAtTop());
        SmartDashboard.putBoolean("IntakeDrop/AtBottom", isAtBottom());
    }
}