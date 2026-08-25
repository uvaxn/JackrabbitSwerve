package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

/**
 * A drop-in replacement for {@link edu.wpi.first.math.filter.SlewRateLimiter} whose
 * acceleration cap tapers off as the output approaches maxOutput, instead of being a flat
 * rate the whole way. Reproduces the "quick punch off the line, soft taper near top speed"
 * feel from MoSim's DriveController.applyWheelForceAtContact / BuildFalloffLookupTable.
 *
 * Real swerve code commands velocity directly, it doesn't have a continuous "current robot
 * speed" the way a Rigidbody-driven sim does, so this uses the limiter's own ramped output
 * as that proxy - close enough for feel, and it means the curve reacts to what the robot is
 * actually being told to do, not just what the joystick says this instant.
 *
 * Only accelerating (moving away from zero) is throttled by the curve. Decelerating back
 * toward zero, or reversing direction, always gets the full accelerationPerSecond rate -
 * letting off the stick or reversing should never feel sluggish, only speeding up should.
 */
public class FalloffRateLimiter {
    private final double accelerationPerSecond; // rate at zero speed, same units as maxOutput/sec
    private final double falloffPercent;        // fraction of accel sacrificed at max output (game default 0.075)
    private final double falloffExponent;       // how late/sharp the taper kicks in (game default 10)
    private final double maxOutput;              // units matching your input/output, e.g. m/s

    private double prevOutput;
    private double prevTimeSeconds;

    public FalloffRateLimiter(
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