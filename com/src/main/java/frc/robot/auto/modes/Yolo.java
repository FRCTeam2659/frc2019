package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.*;
import frc.robot.paths.TrajectoryGenerator;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Superstructure.SuperstructureStates;
import frc.lib.util.DriveSignal;

import java.util.Arrays;

public class Yolo extends AutoModeBase {

    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    private final boolean mStartedLeft;
    private DriveTrajectory mYolo1, mYolo2, mYolo3, mYolo4, mYolo5;
    private final double mYolo1WaitTime, mYolo4WaitTime;

    public Yolo(boolean robotStartedOnLeft) {
        mStartedLeft = robotStartedOnLeft;
        mYolo1 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().sideStartToFrontRocket.get(mStartedLeft), true);
        mYolo2 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().frontRocketToBuffer.get(mStartedLeft));
        mYolo3 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().fronRocketBufferToLoading.get(mStartedLeft));
        mYolo4 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().hatch2ToRocketBuffer.get(mStartedLeft));
        mYolo5 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().bufferToBackRocket.get(mStartedLeft));
        
        mYolo1WaitTime = mTrajectoryGenerator.getTrajectorySet().sideStartToFrontRocket.get(mStartedLeft).getLastState().t() - 1.6;
        mYolo4WaitTime = mTrajectoryGenerator.getTrajectorySet().hatch2ToRocketBuffer.get(mStartedLeft).getLastState().t() - 1.5;
}

    @Override
    public void done() {
        Drive.getInstance().setVelocity(DriveSignal.NEUTRAL, DriveSignal.NEUTRAL);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        // two hatch backside rocket auto
        System.out.println("Running Yolo Program");
        // Start drving path1: scoring the first hatch
        runAction(new ParallelAction(
                Arrays.asList(
                        mYolo1,
                        new SeriesAction(
                                Arrays.asList(
                                        new SetSuperstructure(false),
                                        new WaitAction(1.6), // Don't change this #
                                        new SetSuperstructure(SuperstructureStates.MIDDLE_POSITION),
                                        new WaitAction(mYolo1WaitTime),
                                        new TogglePeg(false)
                                )
                        )
                )
        ));

        // Start driving path2: move to buffer zone
        runAction(mYolo2);

        // Start driving path3; picking the second hatch
        runAction(new SeriesAction(
                Arrays.asList(
                        new SetSuperstructure(SuperstructureStates.LOW_POSITION),
                        mYolo3,
                        new TogglePeg(true)
                )
        ));

        runAction(new ParallelAction(
                Arrays.asList(
                        mYolo4,
                        new SeriesAction(
                                Arrays.asList(
                                        new WaitAction(1.5),
                                        //new LLDrive(),
                                        //new DriveTo(20,-20),
                                        //new DriveTo(20,20)
                                        new SetSuperstructure(SuperstructureStates.MIDDLE_POSITION),
                                        new WaitAction(mYolo4WaitTime),
                                        mYolo5,
                                        new TogglePeg(true)
                                )
                        )
                )
        ));
    }
}
