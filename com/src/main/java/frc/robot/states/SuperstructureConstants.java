package frc.robot.states;

public class SuperstructureConstants {
    //public static final double kWristMinAngle = 40.0;
    //public static final double kWristMaxAngle = 220.0;
    //public static final double kElbowMinAngle = 25.0;
    //public static final double kElbowMaxAngle = 193.0;
    public static final double kElevatorMaxHeight = 41.0;
    public static final double kElevatorMinHeight = 18.5;

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
    public final static double kStowedElevatorHeight = 23;
    public final static double kStowedElbowAngle = 140.0;
    public final static double kStowedWristAngle = 40.0;
    // Climbing
    public final static double kClimbLevel3ElevatorHeight = 41.5;
    public final static double kClimbElbowAngle = 125.0;
    public final static double kClimbWristAngle = 232.0;
    // level 2
    public final static double kClimbLevel2ElevatorHeight = 36;

    // Elevator Heights.
    //hatch
    public static final double kElevatorLowHatchHeight = 21.0; //21
    public static final int kElevatorMiddleHatchHeight = 41; //39
    public static final int kElevatorIntakeHatchHeight = 27;
    //cargo
    public static final int kElevatorIntakeCargoHeight = 25;
    public static final int kElevatorCargoShipHeight = 33;
    public static final int kElevatorMiddleCargoHeight = 30; //29
    public static final int kElevatorHighCargoHeight = 41;
    public static final int kElevatorBackCargoHeight = 41;

    // Elbow Angles.
    //hatch
    public static final int kElbowLowHatchAngle = 100; //125
    public static final int kElbowMiddleHatchAngle = 65; //60
    public static final int kElbowIntakeHatchAngle = 130;
    //cargo
    public static final int kElbowIntakeCargoAngle = 143;
    public static final int kElbowCargoShipAngle = 130;
    public static final int kElbowMiddleCargoAngle = 50;
    public static final int kElbowHighCargoAngle = 15;
    public static final int kElbowBackCargoAngle = 0;

    // Wrist Angles refrencing intake not peg.
    //hatch
    public static final int kWristVisionAngle = 50;
    public static final int kWristLowHatchAngle = 180 - kElbowLowHatchAngle; //55
    public static final int kWristMiddleHatchAngle = 180 - kElbowMiddleHatchAngle;
    public static final int kWristIntakeHatchAngle = 180 - kElbowIntakeHatchAngle;
    //cargo
    public static final int kWristIntakeCargoAngle = 130;
    public static final int kWristCargoShipAngle = 180 - kElbowCargoShipAngle; 
    public static final int kWristMiddleCargoAngle = 180 - kElbowMiddleCargoAngle + 1;
    public static final int kWristHighCargoAngle = 145;
    public static final int kWristBackCargoAngle = 50;
}