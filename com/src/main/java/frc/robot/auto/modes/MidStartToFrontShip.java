package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.*;
import frc.robot.paths.TrajectoryGenerator;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Superstructure.SuperstructureStates;
import frc.lib.util.DriveSignal;

import java.util.Arrays;

public class MidStartToFrontShip extends AutoModeBase {

    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    private final boolean mStartedLeft;
    private final DriveTrajectory mMidlash1;
    private final double mMidlash1WaitTime;

    public MidStartToFrontShip(boolean robotStartedOnLeft) {
        mStartedLeft = robotStartedOnLeft;
        mMidlash1 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().midStartToNearMiddleShip.get(mStartedLeft), true);
        //mMidlash2 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().nearMiddleShipToBuffer.get(mStartedLeft));
        //mMidlash3 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().bufferToHatch2.get(mStartedLeft));
        //mMidlash4 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().hatch2ToRocketBuffer.get(mStartedLeft));
        //mMidlash5 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().bufferToBackRocket.get(mStartedLeft));

        mMidlash1WaitTime = mTrajectoryGenerator.getTrajectorySet().midStartToNearMiddleShip.get(mStartedLeft).getLastState().t() - 0.6;
    }

    @Override
    public void done() {
        Drive.getInstance().setVelocity(DriveSignal.NEUTRAL, DriveSignal.NEUTRAL);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        // two hatch middle auto
        System.out.println("Running Midlash Program");
        // Start drving path1: scoring the first hatch
        runAction(new ParallelAction(
                Arrays.asList(
                        mMidlash1,
                        new SeriesAction(
                                Arrays.asList(
                                        new SetSuperstructure(false),
                                        new WaitAction(0.6), // Don't change this #
                                        new SetSuperstructure(SuperstructureStates.LOW_POSITION)
                                        //new WaitAction(mMidlash1WaitTime),
                                        //new TogglePeg(false)
                                )
                        )
                )
        ));

        // Start driving path2: move to buffer zone
        /*runAction(mMidlash2);

        // Start driving path3; picking the second hatch
        runAction(new SeriesAction(
                Arrays.asList(
                        mMidlash3,
                        new TogglePeg(true)
                )
        ));

        // Scoring the 2nd hatch woohoo
        runAction(new SeriesAction(
                Arrays.asList(
                        mMidlash4,
                        new LLDrive()
                        //new DriveTo(20,-20),
                        //new DriveTo(20,20)
                        //mMidlash5
                        //new TogglePeg(false)
                )
        ));*/
    }
}
