package frc.robot.auto.modes;

import frc.robot.RobotState;
import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.*;
import frc.robot.paths.TrajectoryGenerator;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Superstructure.SuperstructureStates;
import frc.lib.geometry.Pose2d;
import frc.lib.trajectory.timing.CentripetalAccelerationConstraint;
import frc.lib.util.DriveSignal;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Auteleop extends AutoModeBase {

    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    private final Vision mVision = Vision.getInstance();

    private DriveTrajectory mAuteleopTrajectory;

    public Auteleop() {
        List<Pose2d> waypoints = new ArrayList<>();
        waypoints.add(RobotState.getInstance().getLatestFieldToVehicle().getValue());
        waypoints.add(mVision.getCoordinate());
        mAuteleopTrajectory = new DriveTrajectory(mTrajectoryGenerator.generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(100.0)),60.0, 100.0, 9.0), true);
    }

    @Override
    public void done() {
        Drive.getInstance().setVelocity(DriveSignal.NEUTRAL, DriveSignal.NEUTRAL);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running the Great Auteleop Program");
        // Start drving
        runAction(new ParallelAction(
                Arrays.asList(
                        mAuteleopTrajectory,
                        new SeriesAction(
                                Arrays.asList(
                                        new SetSuperstructure(SuperstructureStates.LOW_POSITION),
                                        new TogglePeg(false)
                                )
                        )
                )
        ));
        //score when the path is finished
    }
}
