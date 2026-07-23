package frc.robot.subsystems.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Variables;
import frc.robot.subsystems.EaseofLife;

/**
 * Ground intake rollers only 
 * {@link IntakeDropSubsystem}, driven by Motion Magic instead of an open loop timed move.
 */
public class IntakeSubsystem extends SubsystemBase {

    private final TalonFX intakeMotor;
    private final EaseofLife MotorMode;


    // magnitude only now, 0.0 to 1.0, direction comes from the reverse flag passed to setSpeed()
    private static final double INTAKE_COLLECT_SPEED = 0.6; // collecting from ground

    public IntakeSubsystem(TalonFX intakeMotor, EaseofLife EaseOfLife) {
        this.intakeMotor = intakeMotor;
        this.MotorMode = EaseOfLife;
        MotorMode.configureVelocityControl(intakeMotor);
    }

    public void start() {
        MotorMode.setSpeed(intakeMotor, INTAKE_COLLECT_SPEED, true);

        Variables.requestSpeedLimit("intake", 0.45);
    }

    public void stop() {
        MotorMode.stop(intakeMotor);
        Variables.clearSpeedLimit("intake");
    }

    @Override
    public void periodic() {}
}