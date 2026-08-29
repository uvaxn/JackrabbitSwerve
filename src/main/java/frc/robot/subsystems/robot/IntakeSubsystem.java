package frc.robot.subsystems.robot;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.DriveInputs;
import frc.robot.subsystems.EaseofLife;
    import edu.wpi.first.wpilibj.Timer;
public class IntakeSubsystem extends SubsystemBase {

    private final TalonFX intakeMotor;
    private final TalonFX dropMotor;
    private final DigitalInput upperSensor;
    private final DigitalInput lowerSensor;
    EaseofLife MotorMode;
    private static final double DROP_SPEED = 0.15;
    private static final double LIFT_SPEED = 0.15;

    private static final double AGITATE_DROP_SPEED = 0.15;
    private static final double AGITATE_LIFT_SPEED = 0.15;

    private static final double INTAKE_COLLECT_SPEED = -0.8; // collecting from ground
    private static final double INTAKE_FEED_SPEED = -0.5; // for pushing balls to shooter
    private final CoastOut    coastOut    = new CoastOut();
    private final StaticBrake staticBrake = new StaticBrake();

    private enum DropState { IDLE, MOVING_DOWN, MOVING_UP }

    private final Timer bounceTimer = new Timer();

    private static final double BOUNCE_UP_TIME = 0.3; // time held at top

    private enum BounceState {
        OFF,
        GOING_UP,
        GOING_DOWN
    }

    private BounceState bounceState = BounceState.OFF;
    private boolean bouncing = false;
    private DropState state = DropState.IDLE;
    // what "Seeded" means in these variables is basically has it set off the top sensor (it stores that) 
    private boolean hasSeededTop    = true; // the nail is somewhat bent at the top, so it wont set off the top sensor too well. Best to leave this true.
    // same thing except for it's the bottom sensor
    private boolean hasSeededBottom = false;



    public IntakeSubsystem(TalonFX intakeMotor, TalonFX dropMotor,
                        DigitalInput upperSensor, DigitalInput lowerSensor, EaseofLife EaseOfLife, DriveInputs DriveInputs) {
        this.intakeMotor = intakeMotor;
        this.dropMotor   = dropMotor;
        this.upperSensor = upperSensor;
        this.lowerSensor = lowerSensor;
        this.MotorMode = EaseOfLife;     
        dropMotor.setPosition(0.0);
        dropMotor.setControl(staticBrake);

    }
    public void requestDown() {
        if (isAtBottom() && !edu.wpi.first.wpilibj.RobotBase.isSimulation()) return;
        state = DropState.MOVING_DOWN;

    }
    public void requestUp() {
        if (isAtTop() && !edu.wpi.first.wpilibj.RobotBase.isSimulation()) return;
        state = DropState.MOVING_UP;

    }
    public void startBounce() {
        bouncing = true;
        bounceState = BounceState.GOING_UP;
        MotorMode.setSpeed(intakeMotor, INTAKE_FEED_SPEED);
        requestUp();
        bounceTimer.restart();
    }

    public void stopBounce() {
        bouncing = false;
        bounceState = BounceState.OFF;
        MotorMode.setSpeed(intakeMotor, 0);
        bounceTimer.stop();
        bounceTimer.reset();
        // return the arm to the bottom at the normal drop speed.
        if (!isAtBottom()) {
            requestDown();
        }
    }
    private void updateBounce() {
            if (!bouncing) return;

            switch (bounceState) {
                case GOING_UP -> {
                    // Keep going up until the timer expires
                    if (bounceTimer.hasElapsed(BOUNCE_UP_TIME)) {
                        requestDown();
                        bounceState = BounceState.GOING_DOWN;
                    }
                }
                case GOING_DOWN -> {
                    // Once bottom sensor is hit, go back up
                    if (isAtBottom()) {
                        requestUp();
                        bounceTimer.restart();
                        bounceState = BounceState.GOING_UP;
                    }
                }
                case OFF -> {}
            }
        }
    public void start() {
        MotorMode.setSpeed(intakeMotor, INTAKE_COLLECT_SPEED);
    }
    public void stop() {
        MotorMode.setSpeed(intakeMotor, 0);
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
    /** @return true when the lower sensor has been set off. */
    public boolean isAtBottom() { return !lowerSensor.get(); }
    /** @return true when the upper sensor has been set off. */
    public boolean isAtTop()    { return !upperSensor.get(); }

    @Override
    public void periodic() {
        if (isAtTop() && !hasSeededTop) {
            hasSeededTop    = true;
            hasSeededBottom = false;
        }
        if (isAtBottom() && !hasSeededBottom) {
            hasSeededBottom = true;
            hasSeededTop    = false;
        }
        
        switch (state) {
            case MOVING_DOWN -> {
                if (isAtBottom()) {
                    dropMotor.setControl(coastOut);
                    state = DropState.IDLE;
                } else {
                    double speed = bouncing ? AGITATE_DROP_SPEED : DROP_SPEED;
                    dropMotor.set(-speed);
                }
            }

            case MOVING_UP -> {
                if (isAtTop()) {
                    dropMotor.setControl(staticBrake);
                    state = DropState.IDLE;
                } else {
                    double speed = bouncing ? AGITATE_LIFT_SPEED : LIFT_SPEED;
                    dropMotor.set(speed);
                }
            }

            case IDLE -> {}
        }
        updateBounce();
    }
}