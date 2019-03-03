package frc.robot;

/**
 * A list of constants used by the rest of the robot code. This include physics constants as well as constants
 * determined through calibrations.
 */
public class Constants {
    public static final double kLooperDt = 0.01;

    /* ROBOT PHYSICAL CONSTANTS */

    // Wheels
    public static final double kDriveWheelTrackWidthInches = 26;
    public static final double kDriveWheelDiameterInches = 6.25;
    public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
    public static final double kTrackScrubFactor = 1.0;  // Tune me!

    // Tuned dynamics
    public static final double kRobotLinearInertia = 60.0;  // kg TODO tune
    public static final double kRobotAngularInertia = 10.0;  // kg m^2 TODO tune
    public static final double kRobotAngularDrag = 12.0;  // N*m / (rad/sec) TODO tune
    public static final double kDriveVIntercept = 1.055;  // V
    public static final double kDriveKv = 0.135;  // V per rad/s
    public static final double kDriveKa = 0.012;  // V per rad/s^2

    // Geometry
    public static final double kCenterToFrontBumperDistance = 38.25 / 2.0;
    public static final double kCenterToRearBumperDistance = 38.25 / 2.0;
    public static final double kCenterToSideBumperDistance = 33.75 / 2.0;
    /* CONTROL LOOP GAINS */

    // Gearing and mechanical constants.
    public static final double kDriveDownShiftVelocity = 9.5 * 12.0;  // inches per second
    public static final double kDriveDownShiftAngularVelocity = Math.PI / 2.0; // rad/sec
    public static final double kDriveUpShiftVelocity = 11.0 * 12.0;  // inches per second

    public static final double kPathKX = 4.0;  // units/s per unit of error
    public static final double kPathLookaheadTime = 0.4;  // seconds to look ahead along the path for steering
    public static final double kPathMinLookaheadDistance = 24.0;  // inches

    // PID gains for drive velocity loop (LOW GEAR)
    // Units: setpoint, error, and output are in ticks per second.
    public static final double kDriveLowGearVelocityKp = 0.9;
    public static final double kDriveLowGearVelocityKi = 0.0;
    public static final double kDriveLowGearVelocityKd = 10.0;
    public static final double kDriveLowGearVelocityKf = 0.0;
    public static final int kDriveLowGearVelocityIZone = 0;
    public static final double kDriveVoltageRampRate = 0.0;

    public static final double kDiagonalView = Math.toRadians(68.5);
    public static final double kScreenWidth = 480;
    public static final double kScreenHeight = 270;
    public static final double kCenterScreenWidth = kScreenWidth/2 - 0.5;
    public static final double kCenterScreenHeight = kScreenHeight/2 - 0.5;
    public static final int kHorizontalAspect = 16;
    public static final int kVerticalAspect = 9;
    public static final double kDiagonalAspect = Math.hypot(16, 9);
    public static final double kHorizontalView = Math.atan(Math.tan(kDiagonalView/2)*(kHorizontalAspect/kDiagonalAspect))*2;
    public static final double kVerticalView = Math.atan(Math.tan(kDiagonalView/2)*(kVerticalAspect/kDiagonalAspect))*2;
    public static final double H_FOCAL_LENGTH = kScreenWidth / (2*Math.tan((kHorizontalView/2)));
    public static final double V_FOCAL_LENGTH = kScreenHeight / (2*Math.tan((kVerticalView/2)));
    public static final double kVisionTapeWidth = 15.8; //inches

    // PID gains for elevator velocity loop (HIGH GEAR)
    // Units: setpoint, error, and output are in native units per 100ms.
    // Elevator encoder is CTRE mag encoder which is 4096 native units per revolution.
    public static final double kElevatorHighGearKp = 0.4;
    public static final double kElevatorHighGearKi = 0.0;
    public static final double kElevatorHighGearKd = 4.0;
    public static final double kElevatorHighGearKf = 0.682; // lower speed:  0.06;
    public static final double kElevatorJogKp = 0.3;
    public static final double kElevatorJogKd = 3.0;
    public static final double kElevatorFeedforwardNoCube = -0.06;//33000;
    public static final double kElevatorFeedforwardWithCube = -0.07;//33000;

    public static final int kElevatorHighGearDeadband = 0;
    public static final int kElevatorHighGearCruiseVelocity = 1500;
    public static final int kElevatorHighGearAcceleration = 5000;
    public static final double kElevatorEpsilon = 1.0;//33000;
    public static final double kElevatorRampRate = 0.1;
    
    // gains for elbow
    public static final double kElbowKp = 1.0;
    public static final double kElbowKi = 0.0;
    public static final double kElbowKd = 50.0;
    public static final double kElbowKf = 1.05;
    public static final double kElbowJogKp = 2.0;
    public static final double kElbowJogKd = 40.0;

    public static final int kElbowCruiseVelocity = 1250; //todo: tune me
    public static final int kElbowAcceleration = 2500; //2000 //todo: tune me
    public static final double kElbowRampRate = 0.005;
    public static final double kAutoelbowRampRate = 0.01;

    // gains for wrist
    public static final double kWristKp = 3.0;
    public static final double kWristKi = 0.0;
    public static final double kWristKd = 50.0;
    public static final double kWristKf = 1.05;
    public static final double kWristJogKp = 2.0;
    public static final double kWristJogKd = 40.0;
    //public static final double kWristKaWithCube = 0.006;
    public static final double kWristKa = 0.003;
    public static final double kWristKfMultiplier = 0.1;
    public static final double kWristElevatorAccelerationMultiplier = -1.0;
    public static final double kWristEpsilon = 2.0;

    public static final int kWristDeadband = 5; //todo: tune me
    public static final int kWristCruiseVelocity = 2500; //todo: tune me
    public static final int kWristAcceleration = 2500; //2000 //todo: tune me
    public static final double kWristRampRate = 0.001;
    public static final double kAutoWristRampRate = 0.01;

    // gains for hatch intake joint
    public static final double kHatchJointKp = 3.0;
    public static final double kHatchJointKi = 0.0;
    public static final double kHatchJointKd = 50.0;
    public static final double kHatchJointKf = 0.0;
    public static final int kHatchJointCruiseVelocity = 2000; //todo: tune me
    public static final int kHatchJointAcceleration = 2500;

    // Do not change anything after this line unless you rewire the robot and
    // update the spreadsheet!
    // Port assignments should match up with the spreadsheet here:
    // https://docs.google.com/spreadsheets/d/179YszqnEWPWInuHUrYJnYL48LUL7LUhZrnvmNu1kujE/edit#gid=0

    /* I/O */
    // (Note that if multiple talons are dedicated to a mechanism, any sensors
    // are attached to the master)

    public static final int kCANTimeoutMs = 10; //use for on the fly updates
    public static final int kLongCANTimeoutMs = 100; //use for constructors

    // Drive
    public static final int kLeftDriveMasterId = 0;
    public static final int kLeftDriveSlaveAId = 1;
    public static final int kLeftDriveSlaveBId = 2;
    public static final int kRightDriveMasterId = 9;
    public static final int kRightDriveSlaveAId = 8;
    public static final int kRightDriveSlaveBId = 7;

    // Elevator
    public static final int kElevatorMasterId = 3;

    // Elbow
    public static final int kElbowMasterId = 4;
    // Wrist
    public static final int kWristMasterId = 5;

    // Intake
    public static final int kIntakeMasterId = 0; // VictorSPX

    // Hatch grabber
    public static final int kHatchIntakeMasterId = 1; // VictorSRX
    public static final int kHatchJointMasterId = 6; // TalonSRX

    // Solenoids
    public static final int kShifterSolenoidId = 4;
    public static final int kIntakeSolenoidId = 6;

    // Sensors
    public static final int kPingChannel = 0;
    public static final int kEchoChannel = 0;
}
