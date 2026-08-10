package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;

/**
 *
 * By SysId: run a Phoenix 6 SysId characterization on the real mechanism. It reports kS, kV,
 * and kA directly (kG too, for arm mechanisms, if GravityType is set before the run). Still
 * tune kP, kI, kD by hand afterward, SysId does not measure those.
 *
 * important!!! vvvvvvvvvvv
 * TODO: By hand: raise kS from 0 until the motor just barely starts moving. Estimate kV from top
 * speed divided by the voltage that gets you there. Leave kA at 0 to start. Raise kP until it
 * tracks its target without much overshoot, add kD if it oscillates, leave kI at 0 unless
 * there is steady state error kP alone can't fix.
 */
public final class MotorGains {
    private MotorGains() {}

    /** Position control for a mechanism fighting gravity, like an arm. */
    public record PIDSVAG(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
        public Slot0Configs slot0(GravityTypeValue gravityType) {
            return new Slot0Configs()
                .withKP(kP).withKI(kI).withKD(kD)
                .withKS(kS).withKV(kV).withKA(kA)
                .withKG(kG).withGravityType(gravityType);
        }
    }

    /** Velocity control for a spinning mechanism with no gravity load. */
    public record PIDSV(double kP, double kI, double kD, double kS, double kV) {
        public Slot0Configs slot0() {
            return new Slot0Configs().withKP(kP).withKI(kI).withKD(kD).withKS(kS).withKV(kV);
        }
    }

    public static final PIDSVAG INTAKE_DROP  = new PIDSVAG(5.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0);

    public static final PIDSV SHOOTER        = new PIDSV(0.1, 0.0, 0.0, 0.5, 0.11);

    public static final PIDSV INTAKE_ROLLERS = new PIDSV(0.1, 0.0, 0.0, 0.5, 0.11);
    public static final PIDSV LOWER_FEED     = new PIDSV(0.1, 0.0, 0.0, 0.5, 0.11);
    public static final PIDSV UPPER_FEED     = new PIDSV(0.1, 0.0, 0.0, 0.5, 0.11);
}
