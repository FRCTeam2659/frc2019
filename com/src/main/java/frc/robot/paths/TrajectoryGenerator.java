package frc.robot.paths;

import frc.robot.RobotState;
import frc.robot.planners.DriveMotionPlanner;
import frc.robot.subsystems.Drive;
import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Pose2dWithCurvature;
import frc.lib.geometry.Rotation2d;
import frc.lib.geometry.Translation2d;
import frc.lib.trajectory.Trajectory;
import frc.lib.trajectory.TrajectoryUtil;
import frc.lib.trajectory.timing.CentripetalAccelerationConstraint;
import frc.lib.trajectory.timing.TimedState;
import frc.lib.trajectory.timing.TimingConstraint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TrajectoryGenerator {
    private static final double kMaxVelocity = 115.0;
    private static final double kMaxAccel = 100.0; //130
    private static final double kMaxCentripetalAccelElevatorDown = 115.0;
    private static final double kMaxCentripetalAccel = 110.0; //100
    private static final double kMaxVoltage = 9.0;

    private static final double kMedianVelocity = 86.0;
    private static final double kMinVelocity = 66.0; // less the better
    private static final double kMinAccel = 80;

    private static TrajectoryGenerator mInstance = new TrajectoryGenerator();
    private final DriveMotionPlanner mMotionPlanner;
    private TrajectorySet mTrajectorySet = null;

    public static TrajectoryGenerator getInstance() {
        return mInstance;
    }
    private TrajectoryGenerator() {
        mMotionPlanner = new DriveMotionPlanner();
    }
    public void generateTrajectories() {
        if (mTrajectorySet == null) {
            System.out.println("Generating trajectories...");
            mTrajectorySet = new TrajectorySet();
            System.out.println("Finished trajectory generation");
        }
    }
    public TrajectorySet getTrajectorySet() {
        return mTrajectorySet;
    }
    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_voltage) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, max_vel, max_accel, max_voltage);
    }
    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double start_vel,  // inches/s
            double end_vel,  // inches/s
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_voltage) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, start_vel, end_vel, max_vel, max_accel, max_voltage);
    }

    // CRITICAL POSES
    // Origin is the center of the robot when the robot is placed against the middle of the alliance station wall.
    // +x is towards the center of the field.
    // +y is to the left.
    // ALL POSES DEFINED FOR THE CASE THAT ROBOT STARTS ON RIGHT! (mirrored about +x axis for LEFT)
    public static final Pose2d kSideStartPose = new Pose2d(64.0, -46.5, Rotation2d.fromDegrees(0.0)); //24 the real one is 68, but the hypotenuse shall be counted
    public static final Pose2d kLevel2SideStartPose = new Pose2d(25.0, -46.5, Rotation2d.fromDegrees(0.0));
    public static final Pose2d kLevel2BufferPose = new Pose2d(64.0, -46.5, Rotation2d.fromDegrees(0.0));
    public static final Pose2d kCenterStartPose = new Pose2d(64.0, -6.5, Rotation2d.fromDegrees(0.0)); //+132.25 is front cargo ship
    public static final Pose2d kSideStartBackPose = new Pose2d(29.0, -39.5, Rotation2d.fromDegrees(180.0));
    public static final Pose2d kCargoShipNearMiddleSlot = new Pose2d(new Translation2d(165.0, -10.9), Rotation2d.fromDegrees(0.0)); //199.86, -10.9
    public static final Pose2d kCargoShipFarMiddleSlot = new Pose2d(new Translation2d(140.0, 11.0), Rotation2d.fromDegrees(0.0)); //buffer zone actually
    public static final Pose2d kCargoShipSlot1 = new Pose2d(new Translation2d(256.6, -49.0), Rotation2d.fromDegrees(90.0));
    public static final Pose2d kCargoShipSlot2 = new Pose2d(new Translation2d(281.0, -49.0), Rotation2d.fromDegrees(90.0));
    //public static final Pose2d kBackRocketPose = new Pose2d(new Translation2d(250.0, -128.0), Rotation2d.fromDegrees(210.0)); //technically 261.32
    public static final Pose2d kBackRocketPose = new Pose2d(new Translation2d(275.0, -120.0), Rotation2d.fromDegrees(155.0));
    public static final Pose2d kFrontRocketPose = new Pose2d(new Translation2d(195.0, -124.0), Rotation2d.fromDegrees(-30.0));

    public static final Pose2d kCargoShipSlot1Buffer = new Pose2d(new Translation2d(255.0, -105.0), Rotation2d.fromDegrees(90.0)); //260
    public static final Pose2d kRocketBufferPose = new Pose2d(new Translation2d(286.0, -101.0), Rotation2d.fromDegrees(209.0)); // 288,-75,230
    public static final Pose2d kFrontRocketBufferPose = new Pose2d(new Translation2d(140.0, -80.0), Rotation2d.fromDegrees(260.0)); // 288,-75,230
    public static final Pose2d kHatchBufferPose = new Pose2d(new Translation2d(130.0, -10.9), Rotation2d.fromDegrees(0.0));
    public static final Pose2d kIntakeCargo1Pose = new Pose2d(new Translation2d(60.0, -93.0), Rotation2d.fromDegrees(170.0));
    public static final Pose2d kHatchLoadingPose = new Pose2d(new Translation2d(17.5, -135.0), Rotation2d.fromDegrees(180.0));
    Pose2d odometry = RobotState.getInstance().getLatestFieldToVehicle().getValue();

    public class TrajectorySet {
        public class MirroredTrajectory {
            public MirroredTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> right) {
                this.right = right;
                this.left = TrajectoryUtil.mirrorTimed(right);
            }

            public Trajectory<TimedState <Pose2dWithCurvature>> get(boolean left) {
                return left ? this.left : this.right;
            }

            public final Trajectory<TimedState<Pose2dWithCurvature>> left;
            public final Trajectory<TimedState<Pose2dWithCurvature>> right;
        }

        // Yolo!
        public final MirroredTrajectory midStartToBackRocket;
        public final MirroredTrajectory backRocketToBuffer;
        public final MirroredTrajectory rocketBufferToHatch2;
        public final MirroredTrajectory sideStartToNearMiddleShip;
        public final MirroredTrajectory nearMiddleShipToBuffer;
        //public final MirroredTrajectory bufferToCargo1;
        //public final MirroredTrajectory cargo1ToSlot1;
        // two tocket
        public final MirroredTrajectory sideStartToFrontRocket;
        public final MirroredTrajectory frontRocketToBuffer;
        public final MirroredTrajectory fronRocketBufferToLoading;

        // Frontlash!
        public final MirroredTrajectory bufferToHatch2;
        public final MirroredTrajectory hatch2ToSlot1;
        public final MirroredTrajectory hatch2ToFarMiddleShip;
        public final MirroredTrajectory lastTry;

        // Midlash!
        public final MirroredTrajectory midStartToNearMiddleShip;
        public final MirroredTrajectory hatch2ToRocketBuffer;
        //public final MirroredTrajectory bufferToBackRocket;

        // Backlash!
        public final MirroredTrajectory side2StartToBuffer;
        public final MirroredTrajectory bufferToBackRocket;
        public final MirroredTrajectory backRocketToLoading;
        public final MirroredTrajectory loadingToBackRocket;


        private TrajectorySet() {
            // Yolo!
            midStartToBackRocket = new MirroredTrajectory(getMidStartToBackRocket());
            backRocketToBuffer = new MirroredTrajectory(getBackRocketToBuffer());
            rocketBufferToHatch2 = new MirroredTrajectory(getRocketBufferToHatch2());
            sideStartToNearMiddleShip = new MirroredTrajectory(getSideStartToNearMiddleShip());
            nearMiddleShipToBuffer = new MirroredTrajectory(getNearMiddleShipToBuffer());
            //bufferToCargo1 = new MirroredTrajectory(getBufferToCargo1());
            //cargo1ToSlot1 = new MirroredTrajectory(getCargo1ToSlot1());
            sideStartToFrontRocket = new MirroredTrajectory(getSideStartToFrontRocket());
            frontRocketToBuffer = new MirroredTrajectory(getFrontRocketToBuffer());
            fronRocketBufferToLoading = new MirroredTrajectory(getFrontRocketBufferToLoading());
            
            // Frontlash!
            bufferToHatch2 = new MirroredTrajectory(getBufferToHatch2());
            hatch2ToSlot1 = new MirroredTrajectory(getHatch2ToShip1());
            hatch2ToFarMiddleShip = new MirroredTrajectory(getHatch2ToFarMiddleShip());
            lastTry = new MirroredTrajectory(lastTry());
            
            // Midlash!
            midStartToNearMiddleShip = new MirroredTrajectory(getMidStartToNearShip());
            hatch2ToRocketBuffer = new MirroredTrajectory(getHatch2ToRocketBuffer());
            //bufferToBackRocket = new MirroredTrajectory(getBufferToBackRocket());
            // new stuff
            side2StartToBuffer = new MirroredTrajectory(getSide2StartToBuffer());
            bufferToBackRocket = new MirroredTrajectory(getBufferToBackRocket());
            backRocketToLoading = new MirroredTrajectory(getBackRocketToLoading());
            loadingToBackRocket = new MirroredTrajectory(getLoadingToBackRocket());
        }

        // 1 hatch 1 cargo yolo auto
        private Trajectory<TimedState<Pose2dWithCurvature>> getMidStartToBackRocket() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kCenterStartPose);
            waypoints.add(new Pose2d(new Translation2d(220.0, -50.0), Rotation2d.fromDegrees(-24.0)));
            waypoints.add(kBackRocketPose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMinVelocity, kMinAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getBackRocketToBuffer() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kBackRocketPose);
            waypoints.add(kRocketBufferPose);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, kMinAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getRocketBufferToHatch2() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRocketBufferPose);
            waypoints.add(new Pose2d(new Translation2d(220.0, -105.0), Rotation2d.fromDegrees(186.0)));
            waypoints.add(kHatchLoadingPose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMinVelocity, kMinAccel, kMaxVoltage);
        }
        
        private Trajectory<TimedState<Pose2dWithCurvature>> getSideStartToNearMiddleShip() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLevel2SideStartPose);//kSideStartPose
            waypoints.add(kCargoShipNearMiddleSlot);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMinVelocity, kMinAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getNearMiddleShipToBuffer() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kCargoShipNearMiddleSlot);
            waypoints.add(kHatchBufferPose);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getSideStartToFrontRocket() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLevel2BufferPose);//kSideStartPose
            waypoints.add(kFrontRocketPose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    60, 60, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getFrontRocketToBuffer() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kFrontRocketPose);
            waypoints.add(kFrontRocketBufferPose);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMedianVelocity, kMinAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getFrontRocketBufferToLoading() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kFrontRocketBufferPose);
            waypoints.add(kHatchLoadingPose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMinVelocity, kMinAccel, kMaxVoltage);
        }
        /*private Trajectory<TimedState<Pose2dWithCurvature>> getBufferToCargo1() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kHatchBufferPose);
            waypoints.add(new Pose2d(new Translation2d(170.0, -55.0), Rotation2d.fromDegrees(260.0)));
            waypoints.add(kIntakeCargo1Pose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getCargo1ToSlot1() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kIntakeCargo1Pose);
            waypoints.add(new Pose2d(new Translation2d(190.0, -105.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(kCargoShipSlot1);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, kMaxAccel, kMaxVoltage);
        }*/


        // 2 hatch cargo ship frontlash auto
        // the hatch scoring is the same as yolo
        private Trajectory<TimedState<Pose2dWithCurvature>> getBufferToHatch2() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kHatchBufferPose);
            waypoints.add(new Pose2d(new Translation2d(160.0, -85.0), Rotation2d.fromDegrees(230.0)));
            waypoints.add(kHatchLoadingPose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMedianVelocity, kMaxAccel, kMaxVoltage);
        }
        // This is a buffer zone, vision will take care of the rest!
        private Trajectory<TimedState<Pose2dWithCurvature>> getHatch2ToShip1() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kHatchLoadingPose);
            //waypoints.add(kCargoShipSlot1Buffer);
            waypoints.add(new Pose2d(new Translation2d(260.0, -100.0), Rotation2d.fromDegrees(105.0)));

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> lastTry() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(260.0, -100.0), Rotation2d.fromDegrees(105.0)));
            waypoints.add(kCargoShipSlot1);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMedianVelocity, kMaxAccel, kMaxVoltage);
        }
        // Alternate solution. This is a buffer zone, vision will take care of the rest!
        private Trajectory<TimedState<Pose2dWithCurvature>> getHatch2ToFarMiddleShip() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kHatchLoadingPose);
            waypoints.add(kCargoShipFarMiddleSlot);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        // Midlash middle 2 hatch auto
        private Trajectory<TimedState<Pose2dWithCurvature>> getMidStartToNearShip() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kCenterStartPose);
            waypoints.add(kCargoShipNearMiddleSlot);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMinVelocity, kMinAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getHatch2ToRocketBuffer() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kHatchLoadingPose);
            waypoints.add(new Pose2d(new Translation2d(222.0, -104.0), Rotation2d.fromDegrees(180.0))); //228,-106,180
            waypoints.add(kRocketBufferPose);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        /*private Trajectory<TimedState<Pose2dWithCurvature>> getBufferToBackRocket() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRocketBufferPose);
            waypoints.add(kBackRocketPose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMedianVelocity, kMaxAccel, kMaxVoltage);
        }*/

        // two hatch back rocket   
        private Trajectory<TimedState<Pose2dWithCurvature>> getSide2StartToBuffer() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLevel2SideStartPose);
            waypoints.add(kLevel2BufferPose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    50, 60, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getBufferToBackRocket() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLevel2SideStartPose);
            waypoints.add(kBackRocketPose);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getBackRocketToLoading() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(odometry.getTranslation().x(), odometry.getTranslation().y()), odometry.getRotation()));
            waypoints.add(kHatchLoadingPose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    50, kMinAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getLoadingToBackRocket() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kHatchLoadingPose);
            waypoints.add(kBackRocketPose);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    50, kMinAccel, kMaxVoltage);
        }
    }
}
