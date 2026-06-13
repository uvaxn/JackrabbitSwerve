package frc.robot.util;
/*
    StructPublisher<Pose2d>        pose    = table.getStructTopic("Pose", Pose2d.struct).publish();
    StructPublisher<Pose3d>        pose3d  = table.getStructTopic("Pose3d", Pose3d.struct).publish();
    StructPublisher<Translation2d> trans   = table.getStructTopic("Trans", Translation2d.struct).publish();
    StructPublisher<Rotation2d>    rot     = table.getStructTopic("Rot", Rotation2d.struct).publish();
    StructPublisher<ChassisSpeeds> speeds  = table.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
    DoublePublisher       // double
    FloatPublisher        // float
    IntegerPublisher      // long
    BooleanPublisher      // boolean
    StringPublisher       // String
    Array versions
    DoubleArrayPublisher  // double[]
    FloatArrayPublisher   // float[]
    IntegerArrayPublisher // long[]
    BooleanArrayPublisher // boolean[]
    StringArrayPublisher  // String[]
    StructArrayPublisher<Pose2d>   poses   = table.getStructArrayTopic("Poses", Pose2d.struct).publish();
*/
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleArrayPublisher;

public class nt {
    private static final NetworkTableInstance nt = NetworkTableInstance.getDefault();

    private static final NetworkTable table = nt.getTable("Robot");

    // key
    private static final DoublePublisher targetAngle     = table.getDoubleTopic("AutoAlign/TargetAngleDeg").publish();
    // These use the struct publisher pattern

    public static void putTargetAngle(double deg)            { targetAngle.set(deg); }
    // just copy this over and over ig
}
