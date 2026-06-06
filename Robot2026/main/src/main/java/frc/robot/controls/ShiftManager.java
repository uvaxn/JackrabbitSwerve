package frc.robot.controls;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ShiftManager {

    public enum Shift {
        TRANSITION, SHIFT_1, SHIFT_2, SHIFT_3, SHIFT_4, ENDGAME, UNKNOWN
    }

    public static final double TRANSITION_END = 130.0;
    public static final double SHIFT1_END     = 105.0;
    public static final double SHIFT2_END     =  80.0;
    public static final double SHIFT3_END     =  55.0;
    public static final double ENDGAME_START  =  30.0;

    private final StringPublisher  currentShiftPublisher;
    private final DoublePublisher  shiftTimeRemainingPublisher;

    private boolean shift1Triggered  = false;
    private boolean shift2Triggered  = false;
    private boolean shift3Triggered  = false;
    private boolean shift4Triggered  = false;
    private boolean endgameTriggered = false;

    public ShiftManager() {
        var nt = NetworkTableInstance.getDefault();
        currentShiftPublisher       = nt.getStringTopic("EaseofLife/CurrentShift").publish();
        shiftTimeRemainingPublisher = nt.getDoubleTopic("EaseofLife/ShiftTimeRemaining").publish();
    }

    public void teleopInit() {
        shift1Triggered  = false;
        shift2Triggered  = false;
        shift3Triggered  = false;
        shift4Triggered  = false;
        endgameTriggered = false;
    }

    public void periodic() {
        currentShiftPublisher.set(getCurrentShift().name());
        shiftTimeRemainingPublisher.set(getShiftTimeRemaining());
    }

    public Shift getCurrentShift() {
        if (!DriverStation.isTeleopEnabled()) return Shift.UNKNOWN;
        double t = DriverStation.getMatchTime();
        if (t < 0)              return Shift.UNKNOWN;
        if (t > TRANSITION_END) return Shift.TRANSITION;
        if (t > SHIFT1_END)     return Shift.SHIFT_1;
        if (t > SHIFT2_END)     return Shift.SHIFT_2;
        if (t > SHIFT3_END)     return Shift.SHIFT_3;
        if (t > ENDGAME_START)  return Shift.SHIFT_4;
        return Shift.ENDGAME;
    }

    public double getShiftTimeRemaining() {
        if (!DriverStation.isTeleopEnabled()) return 0;
        double t = DriverStation.getMatchTime();
        if (t < 0) return 0;
        return switch (getCurrentShift()) {
            case TRANSITION -> t - TRANSITION_END;
            case SHIFT_1    -> t - SHIFT1_END;
            case SHIFT_2    -> t - SHIFT2_END;
            case SHIFT_3    -> t - SHIFT3_END;
            case SHIFT_4    -> t - ENDGAME_START;
            case ENDGAME    -> t;
            default         -> 0;
        };
    }

    public void onShiftChange(
        Runnable onShift1, Runnable onShift2,
        Runnable onShift3, Runnable onShift4,
        Runnable onEndgame
    ) {
        Shift current = getCurrentShift();
        if (!shift1Triggered  && current == Shift.SHIFT_1)  { shift1Triggered  = true; if (onShift1  != null) onShift1.run();  }
        if (!shift2Triggered  && current == Shift.SHIFT_2)  { shift2Triggered  = true; if (onShift2  != null) onShift2.run();  }
        if (!shift3Triggered  && current == Shift.SHIFT_3)  { shift3Triggered  = true; if (onShift3  != null) onShift3.run();  }
        if (!shift4Triggered  && current == Shift.SHIFT_4)  { shift4Triggered  = true; if (onShift4  != null) onShift4.run();  }
        if (!endgameTriggered && current == Shift.ENDGAME)  { endgameTriggered = true; if (onEndgame != null) onEndgame.run(); }
    }
}