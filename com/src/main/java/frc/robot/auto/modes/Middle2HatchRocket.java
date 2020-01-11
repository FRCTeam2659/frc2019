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
import frc.robot.auto.actions.OverrideWhenTargetPresents;
import frc.robot.auto.actions.ParallelAction;
import frc.robot.auto.actions.SeriesAction;
import frc.robot.auto.actions.SetOpenLoop;
import frc.robot.auto.actions.SetSuperstructure;
import frc.robot.auto.actions.TogglePeg;

public class Middle2HatchRocket extends AutoModeBase {
        private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
        private final boolean mStartedLeft;
        private double sideCoefficient;
        private DriveTrajectory mYolo0, mYolo1, mYolo2, mYolo3;
    
        public Middle2HatchRocket(boolean robotStartedOnLeft) {
            mStartedLeft = robotStartedOnLeft;
            if (robotStartedOnLeft)
                sideCoefficient = -1;
            else
                sideCoefficient = 1;
            //mYolo0 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().side2StartToBuffer.get(mStartedLeft), true);
            mYolo1 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().sideStartToFrontRocket.get(mStartedLeft), true);
            //mYolo2 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().backRocketToLoading.get(mStartedLeft));
            mYolo3 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().loadingToBackRocket.get(mStartedLeft));
        }
    
        @Override
        public void done() {
            Drive.getInstance().setVelocity(DriveSignal.NEUTRAL, DriveSignal.NEUTRAL);
        }
    
        @Override
        protected void routine() throws AutoModeEndedException {
            // two hatch backside rocket auto
            System.out.println("Running Yolo Program");
                
            /*runAction(new SeriesAction(
                Arrays.asList(
                        mYolo0,
                        new SetSuperstructure(false),
                        new SetSuperstructure(SuperstructureStates.MIDDLE_POSITION)
                )
            ));*/
            runAction(new ParallelAction(
                    Arrays.asList(
                            mYolo1,
                            new SeriesAction(
                                    Arrays.asList(   
                                            new SetSuperstructure(false),
                                            new WaitAction(1.0),
                                            new SetSuperstructure(SuperstructureStates.MIDDLE_POSITION),
                                            new WaitAction(1.5),
                                            new OverrideWhenTargetPresents(3),
                                            new WaitAction(0.3),
                                            new TogglePeg(false)
                                    )
                            )
                    )
            ));
     
            runAction(new SeriesAction(
                    Arrays.asList(
                            new DriveTo(-40, -40),
                            new SetSuperstructure(SuperstructureStates.LOW_POSITION),
                            new TurnToAngle(sideCoefficient*(-175), 5, 3),
                            new DriveToVisionTarget(5),
                            new SetSuperstructure(SuperstructureStates.LOW_POSITION)
                            //new WaitAction(0.35),
                            //new SetOpenLoop(0.4, 0.4),
                            //new DriveTo(8, 8),
                            //new TogglePeg(true)
                    )
            ));
            /*List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(RobotState.getInstance().getLatestFieldToVehicle().getValue());
            waypoints.add(TrajectoryGenerator.kBackRocketPose);
            Trajectory<TimedState<Pose2dWithCurvature>> mYolo2Path;
            if (mStartedLeft)
                    mYolo2Path = TrajectoryUtil.mirrorTimed(mTrajectoryGenerator.generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(100.0)), 100.0, 110.0, 9.0));
            else
                    mYolo2Path = mTrajectoryGenerator.generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(100.0)), 100.0, 110.0, 9.0);
            
            mYolo2 = new DriveTrajectory(mYolo2Path);
            runAction(new ParallelAction(
                    Arrays.asList(
                            mYolo2,
                            new SeriesAction(
                                    Arrays.asList(
                                            new WaitAction(0.5),
                                            new SetSuperstructure(SuperstructureStates.MIDDLE_POSITION)
                                    )
                            )
                    )
            ));

            runAction(new SeriesAction(
                    Arrays.asList(
                            new TurnToAngle(-150*sideCoefficient, 5, 3),
                            new DriveToVisionTarget(3),
                            new TogglePeg(false)
                    )
            ));*/
        }
}
