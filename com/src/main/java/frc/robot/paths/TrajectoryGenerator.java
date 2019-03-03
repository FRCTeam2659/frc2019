package frc.robot.paths;

import frc.robot.planners.DriveMotionPlanner;
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
    private static final double kMaxVelocity = 130.0;
    private static final double kMaxAccel = 130.0;
    private static final double kMaxCentripetalAccelElevatorDown = 110.0;
    private static final double kMaxCentripetalAccel = 100.0;
    private static final double kMaxVoltage = 9.0;
    private static final double kFirstPathMaxVoltage = 9.0;
    private static final double kFirstPathMaxAccel = 130.0;
    private static final double kFirstPathMaxVel = 130.0;

    private static final double kSimpleSwitchMaxAccel = 100.0;
    private static final double kSimpleSwitchMaxCentripetalAccel = 80.0;
    private static final double kSimpleSwitchMaxVelocity = 120.0;

    private static final double kIntakeMaxVelocity = 95.0;

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
    public static final Pose2d kSideStartPose = new Pose2d(40.0, -43.0, Rotation2d.fromDegrees(0.0));
    public static final Pose2d kSideStartBackPose = new Pose2d(40.0, -43.0, Rotation2d.fromDegrees(180.0));
    public static final Pose2d kCargoShipNearMiddleSlot = new Pose2d(new Translation2d(198.0, -10.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d kCargoShipFarMiddleSlot = new Pose2d(new Translation2d(140.0, 12.0), Rotation2d.fromDegrees(0.0)); //buffer zone actually
    public static final Pose2d kCargoShipSlot1 = new Pose2d(new Translation2d(260.0, -45.0), Rotation2d.fromDegrees(270.0));
    public static final Pose2d kCargoShipSlot2 = new Pose2d(new Translation2d(281.0, -45.0), Rotation2d.fromDegrees(270.0));

    public static final Pose2d kHatchBufferPose = new Pose2d(new Translation2d(130.0, -10.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d kIntakeCargo1Pose = new Pose2d(new Translation2d(60.0, -93.0), Rotation2d.fromDegrees(170.0));
    public static final Pose2d kHatchLoadingPose = new Pose2d(new Translation2d(20.0, -135.0), Rotation2d.fromDegrees(180.0));

    public class TrajectorySet {
        public class MirroredTrajectory {
            public MirroredTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> right) {
                this.right = right;
                this.left = TrajectoryUtil.mirrorTimed(right);
            }

            public Trajectory<TimedState<Pose2dWithCurvature>> get(boolean left) {
                return left ? this.left : this.right;
            }

            public final Trajectory<TimedState<Pose2dWithCurvature>> left;
            public final Trajectory<TimedState<Pose2dWithCurvature>> right;
        }

        // Yolo!
        public final MirroredTrajectory sideStartToNearMiddleShip;
        public final MirroredTrajectory nearMiddleShipToBuffer;
        public final MirroredTrajectory bufferToCargo1;
        public final MirroredTrajectory cargo1ToSlot1;

        // Frontlash!
        public final MirroredTrajectory bufferToHatch2;
        public final MirroredTrajectory hatch2ToFarMiddleShip;

        // Backlash!
        public final MirroredTrajectory sideStartToShip1;
        public final MirroredTrajectory getSlot1ToCargo1;
        public final MirroredTrajectory getCargo1ToSlot2;


        private TrajectorySet() {
            // Yolo!
            sideStartToNearMiddleShip = new MirroredTrajectory(getSideStartToNearMiddleShip());
            nearMiddleShipToBuffer = new MirroredTrajectory(getNearMiddleShipToBuffer());
            bufferToCargo1 = new MirroredTrajectory(getBufferToCargo1());
            cargo1ToSlot1 = new MirroredTrajectory(getCargo1ToSlot1());
            
            // Frontlash!
            bufferToHatch2 = new MirroredTrajectory(getBufferToHatch2());
            hatch2ToFarMiddleShip = new MirroredTrajectory(getHatch2ToFarMiddleShip());
            
            // Backlash!
            sideStartToShip1 = new MirroredTrajectory(getSideStartToShip1());
            getSlot1ToCargo1 = new MirroredTrajectory(getSlot1ToCargo1());
            getCargo1ToSlot2 = new MirroredTrajectory(getCargo1ToSlot2());    
        }

        // 1 hatch 1 cargo yolo auto
        private Trajectory<TimedState<Pose2dWithCurvature>> getSideStartToNearMiddleShip() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kSideStartPose);
            waypoints.add(kCargoShipNearMiddleSlot);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getNearMiddleShipToBuffer() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kCargoShipNearMiddleSlot);
            waypoints.add(kHatchBufferPose);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getBufferToCargo1() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kHatchBufferPose);
            waypoints.add(new Pose2d(170.0, -55.0, Rotation2d.fromDegrees(260.0)));
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
        }

        // 2 hatch middle cargo ship frontlash auto
        // the hatch scoring is the same as yolo
        private Trajectory<TimedState<Pose2dWithCurvature>> getBufferToHatch2() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kHatchBufferPose);
            waypoints.add(kHatchLoadingPose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        // This is a buffer zone, vision will take care of the rest!
        private Trajectory<TimedState<Pose2dWithCurvature>> getHatch2ToFarMiddleShip() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kHatchLoadingPose);
            waypoints.add(kCargoShipFarMiddleSlot);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        // 2 cargo Backlash auto
        private Trajectory<TimedState<Pose2dWithCurvature>> getSideStartToShip1() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kSideStartBackPose);
            waypoints.add(kCargoShipSlot1);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getSlot1ToCargo1() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kCargoShipSlot1);
            waypoints.add(new Pose2d(new Translation2d(220.0, -105.0), Rotation2d.fromDegrees(190.0)));
            waypoints.add(kIntakeCargo1Pose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getCargo1ToSlot2() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kIntakeCargo1Pose);
            waypoints.add(new Pose2d(new Translation2d(200.0, -105.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(kCargoShipSlot2);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
    }
}
