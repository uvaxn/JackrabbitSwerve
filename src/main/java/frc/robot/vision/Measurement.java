package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/** A single accepted vision pose update, ready to hand to a WPILib pose estimator's
 *  addVisionMeasurement. */
public class Measurement {
    public final Pose2d pose;
    public final double timestamp;
    public final Matrix<N3, N1> standardDeviations;

    public Measurement(Pose2d pose, double timestamp, Matrix<N3, N1> standardDeviations) {
        this.pose = pose;
        this.timestamp = timestamp;
        this.standardDeviations = standardDeviations;
    }
}