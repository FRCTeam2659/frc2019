package frc.robot.auto.modes;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Pose2dWithCurvature;
import frc.lib.trajectory.Trajectory;
import frc.lib.trajectory.TrajectoryUtil;
import frc.lib.trajectory.timing.CentripetalAccelerationConstraint;
import frc.lib.trajectory.timing.TimedState;
import frc.lib.util.DriveSignal;
import frc.robot.RobotState;
import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.TurnToAngle;
import frc.robot.auto.actions.WaitAction;
import frc.robot.paths.TrajectoryGenerator;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Superstructure.SuperstructureStates;
import frc.robot.auto.actions.DriveTo;
import frc.robot.auto.actions.DriveToVisionTarget;
import frc.robot.auto.actions.DriveTrajectory;
import frc.robot.auto.actions.DriveTurnToVisionTarget;
import frc.robot.auto.actions.OverrideWhenTargetPresents;
import frc.robot.auto.actions.ParallelAction;
import frc.robot.auto.actions.SeriesAction;
import frc.robot.auto.actions.SetSuperstructure;
import frc.robot.auto.actions.TogglePeg;

public class TestMode extends AutoModeBase {
        private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
        private final boolean mStartedLeft;
        private DriveTrajectory mYolo0, mYolo1, mYolo2, mYolo3;
    
        public TestMode(boolean robotStartedOnLeft) {
            mStartedLeft = robotStartedOnLeft;
            mYolo0 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().side2StartToBuffer.get(mStartedLeft), true);
            mYolo1 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().bufferToBackRocket.get(mStartedLeft), true);
            //mYolo2 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().backRocketToLoading.get(mStartedLeft));
            //mYolo3 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().loadingToBackRocket.get(mStartedLeft));
        }
    
        @Override
        public void done() {
            Drive.getInstance().setVelocity(DriveSignal.NEUTRAL, DriveSignal.NEUTRAL);
        }
    
        @Override
        protected void routine() throws AutoModeEndedException {
            // two hatch backside rocket auto
            System.out.println("Running Yolo Program");
    
            //runAction(mYolo0);
            runAction(new ParallelAction(
                    Arrays.asList(
                            mYolo1,
                            new SeriesAction(
                                    Arrays.asList(
                                            new SetSuperstructure(false),
                                            new WaitAction(1.2), // Don't change this #
                                            new SetSuperstructure(SuperstructureStates.MIDDLE_POSITION)
                                            //maybe eject first stage too
                                    )
                            )
                    )
            ));
     
            runAction(new SeriesAction(
                    Arrays.asList(
                            new TurnToAngle(30, 4, 3),
                            new DriveToVisionTarget(2),
                            new TogglePeg(false),
                            new DriveTo(-7, -7),
                            new TurnToAngle(-30, 4, 3),
                            new SetSuperstructure(SuperstructureStates.LOW_POSITION)
                    )
            ));
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(RobotState.getInstance().getLatestFieldToVehicle().getValue());
            waypoints.add(TrajectoryGenerator.kHatchLoadingPose);
            Trajectory<TimedState<Pose2dWithCurvature>> mYolo2Path;
            if (mStartedLeft)
                    mYolo2Path = TrajectoryUtil.mirrorTimed(mTrajectoryGenerator.generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(100.0)), 100.0, 100.0, 9.0));
            else
                    mYolo2Path = mTrajectoryGenerator.generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(100.0)), 100.0, 100.0, 9.0);
            
            mYolo2 = new DriveTrajectory(mYolo2Path);
            runAction(new ParallelAction(
                    Arrays.asList(
                            mYolo2
                            /*new SeriesAction(
                                    Arrays.asList(
                                            new WaitAction(1.5),
                                            new OverrideWhenTargetPresents(3), //this include ll drive supposedly
                                            new TogglePeg(true)
                                    )
                            )*/
                    )
            ));
            runAction(new TogglePeg(true));
            List<Pose2d> waypoints2 = new ArrayList<>();
            waypoints2.add(RobotState.getInstance().getLatestFieldToVehicle().getValue());
            waypoints2.add(TrajectoryGenerator.kBackRocketPose);
            Trajectory<TimedState<Pose2dWithCurvature>> mYolo3Path;
            if (mStartedLeft)
                    mYolo3Path = TrajectoryUtil.mirrorTimed(mTrajectoryGenerator.generateTrajectory(true, waypoints2, Arrays.asList(new CentripetalAccelerationConstraint(100.0)), 100.0, 100.0, 9.0));
            else
                    mYolo3Path = mTrajectoryGenerator.generateTrajectory(true, waypoints2, Arrays.asList(new CentripetalAccelerationConstraint(100.0)), 100.0, 100.0, 9.0);
            
            mYolo3 = new DriveTrajectory(mYolo3Path);
    
            runAction(new SeriesAction(
                    Arrays.asList(
                            mYolo3,
                            new TurnToAngle(30, 4, 3),
                            new DriveToVisionTarget(3),
                            new DriveTo(4, 4),
                            new TogglePeg(false)
                    )
            ));
        }
}
