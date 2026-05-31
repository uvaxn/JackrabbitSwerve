package frc.robot.controls;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;

public class EaseofLife {
    public void setSpeed(TalonFX motor, double output) {
        motor.set(MathUtil.clamp(output, -1.0, 1.0));
    }

    public void setVelocity() {

    }
    public void setBrake(TalonFX motor, boolean brake) {
        motor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }
    public boolean isSensorTripped(DigitalInput sensor) {
        return !sensor.get();
    }
}