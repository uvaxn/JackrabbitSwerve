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

    /** Trigger pressed arm goes down, intake spins */
    public void requestDown() {
        if (isAtBottom()) return;
        // intakeMotor.set(0.8);
        state = DropState.MOVING_DOWN;
    }

    /** Trigger released intake stops, arm goes back up */
        public void requestUp() {
            intakeMotor.stopMotor(); // already uncommented, good
            if (isAtTop()) {
                dropMotor.setControl(staticBrake);
                state = DropState.IDLE;
                return;
            }
            state = DropState.MOVING_UP;
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
                    System.out.println("IntakeDrop: Reached bottom.");
                } else {
                    dropMotor.set(-DROP_SPEED);
                }
            }
            case MOVING_UP -> {
                if (isAtTop() || pos <= SOFT_LIMIT_UP) {
                    dropMotor.setControl(staticBrake);
                    state = DropState.IDLE;
                    System.out.println("IntakeDrop: Reached top.");
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