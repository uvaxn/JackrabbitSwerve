package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;


public class DriveEasing {
    private final double accelerationPerSecond; // rate at zero speed, same units as maxOutput/sec
    private final double falloffPercent;        // fraction of accel sacrificed at max output (game default 0.075)
    private final double falloffExponent;       // how late/sharp the taper kicks in (game default 10)
    private final double maxOutput;              // units matching your input/output, e.g. m/s

    private double prevOutput;
    private double prevTimeSeconds;

    public DriveEasing(
            double accelerationPerSecond, double falloffPercent, double falloffExponent, double maxOutput) {
        this.accelerationPerSecond = accelerationPerSecond;
        this.falloffPercent = falloffPercent;
        this.falloffExponent = falloffExponent;
        this.maxOutput = maxOutput;
        this.prevOutput = 0;
        this.prevTimeSeconds = Timer.getFPGATimestamp();
    }

    /** Same call pattern as SlewRateLimiter: feed it a target each loop, get back the
     *  ramped value to actually command. */
    public double calculate(double input) {
        double now = Timer.getFPGATimestamp();
        double dt = now - prevTimeSeconds;
        prevTimeSeconds = now;

        double speedRatio = MathUtil.clamp(Math.abs(prevOutput) / maxOutput, 0.0, 1.0);
        double falloff = Math.pow(1.0 - speedRatio * falloffPercent, falloffExponent);
        double maxDelta = accelerationPerSecond * falloff * dt;

        double error = input - prevOutput;
        boolean accelerating = Math.abs(input) > Math.abs(prevOutput);

        double delta = accelerating
            ? MathUtil.clamp(error, -maxDelta, maxDelta)
            : error; // decel/reversal: no cap, snap straight toward input

        prevOutput += delta;
        return prevOutput;
    }

    /** Resets the ramped state, e.g. after a mode change. Matches SlewRateLimiter's reset(). */
    public void reset(double value) {
        prevOutput = value;
        prevTimeSeconds = Timer.getFPGATimestamp();
    }
}