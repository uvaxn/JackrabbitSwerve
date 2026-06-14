package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import frc.robot.Vars;

public class nt {
    private static final NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
    private static final NetworkTable rbtTable = ntInst.getTable("Robot");
    private static final NetworkTable infoTable = ntInst.getTable("Info");

    // robot
    private static final StringPublisher robotState = rbtTable.getStringTopic("State").publish();

    // autoalign
    private static final DoublePublisher targetAngle = rbtTable.getDoubleTopic("AutoAlign/TargetAngleDeg").publish();

    // ease of life
    private static final StringPublisher currentShift = infoTable.getStringTopic("EaseofLife/CurrentShift").publish();
    private static final DoublePublisher shiftTimeRemaining = infoTable.getDoubleTopic("EaseofLife/ShiftTimeRemaining").publish();
    private static final BooleanPublisher isHubActive = infoTable.getBooleanTopic("EaseofLife/IsHubActive").publish();
    private static final DoublePublisher distToHubPublisher = infoTable.getDoubleTopic("EaseofLife/DistanceToHub").publish();
    private static final DoublePublisher shooterSpeed = infoTable.getDoubleTopic("EaseofLife/ShooterSpeed").publish();

    // PID publishers (initial values)
    private static final DoublePublisher AligntoHubP = rbtTable.getDoubleTopic("EaseofLife/PID/AlignToHubP").publish();
    private static final DoublePublisher AligntoHubI = rbtTable.getDoubleTopic("EaseofLife/PID/AlignToHubI").publish();
    private static final DoublePublisher AligntoHubD = rbtTable.getDoubleTopic("EaseofLife/PID/AlignToHubD").publish();


    // PID subscribers (for live tuning from dashboard)
    private static final DoubleSubscriber alignPSub = rbtTable.getDoubleTopic("EaseofLife/PID/AlignToHubP").subscribe(Vars.AlignToHubP);
    private static final DoubleSubscriber alignISub = rbtTable.getDoubleTopic("EaseofLife/PID/AlignToHubI").subscribe(Vars.AlignToHubI);
    private static final DoubleSubscriber alignDSub = rbtTable.getDoubleTopic("EaseofLife/PID/AlignToHubD").subscribe(Vars.AlignToHubD);

    

    public static void putTargetAngle(double deg)   { targetAngle.set(deg); }
    public static void putCurrentShift(String shift){ currentShift.set(shift); }
    public static void putShiftTime(double time)    { shiftTimeRemaining.set(time); }
    public static void isHubActive(boolean active)  { isHubActive.set(active); }
    public static void putRobotState(String state)  { robotState.set(state); }
    public static void putDisttoHub(double dist)    { distToHubPublisher.set(dist); }
    public static void putShooterspeed(double speed){ shooterSpeed.set(speed); }
    // PID setters
    public static void putP(double P) { AligntoHubP.set(P); }
    public static void putI(double I) { AligntoHubI.set(I); }
    public static void putD(double D) { AligntoHubD.set(D); }

    // PID getters (reads live value from dashboard)
    public static double getAlignP() { return alignPSub.get(); }
    public static double getAlignI() { return alignISub.get(); }
    public static double getAlignD() { return alignDSub.get(); }
}