package frc.robot.states;

public class SuperstructureConstants {
    //public static final double kWristMinAngle = 40.0;
    //public static final double kWristMaxAngle = 220.0;
    //public static final double kElbowMinAngle = 25.0;
    //public static final double kElbowMaxAngle = 193.0;
    public static final double kElevatorMaxHeight = 41.0;
    public static final double kElevatorMinHeight = 18.0;

    //public static final double kElevatorLongRaiseDistance = 28.0;
    //public static final double kElevatorApproachingThreshold = 12.0;

    // This is in inches / ~20ms
    //public final static double kElevatorJogThrottle = 60.0 / 50.0;

    // This is in degrees / ~20ms
    //public final static double kWristJogThrottle = 200.0 / 25.0;

    // In inches, the height to use the kPlacingHighAngle.
    //public final static double kPlacingHighThreshold = 33.0;

    // Presets.
    // Combinations.
    public final static double kStowedElevatorHeight = 25;
    public final static double kStowedElbowAngle = 140.0;
    public final static double kStowedWristAngle = 40.0;

    // Elevator Heights.
    //hatch
    public static final int kElevatorLowHatchHeight = 29; //31
    public static final int kElevatorMiddleHatchHeight = 39;
    public static final int kElevatorIntakeHatchHeight = 29;
    //cargo
    public static final int kElevatorIntakeCargoHeight = 25;
    public static final int kElevatorLowCargoHeight = 25;
    public static final int kElevatorMiddleCargoHeight = 33; //29
    public static final int kElevatorHighCargoHeight = 41;
    public static final int kElevatorBackCargoHeight = 41;

    // Elbow Angles.
    //hatch
    public static final int kElbowLowHatchAngle = 110; //125
    public static final int kElbowMiddleHatchAngle = 50;
    public static final int kElbowIntakeHatchAngle = 130;
    //cargo
    public static final int kElbowIntakeCargoAngle = 140;
    public static final int kElbowLowCargoAngle = 140;
    public static final int kElbowMiddleCargoAngle = 50;
    public static final int kElbowHighCargoAngle = 10;
    public static final int kElbowBackCargoAngle = -5;

    // Wrist Angles refrencing intake not peg.
    //hatch
    public static final int kWristLowHatchAngle = 70; //55
    public static final int kWristMiddleHatchAngle = 130;
    public static final int kWristIntakeHatchAngle = 50;
    //cargo
    public static final int kWristIntakeCargoAngle = 130;
    public static final int kWristLowCargoAngle = 40;
    public static final int kWristMiddleCargoAngle = 130;
    public static final int kWristHighCargoAngle = 145;
    public static final int kWristBackCargoAngle = 50;
}