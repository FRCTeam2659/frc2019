package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.*;
import frc.robot.paths.TrajectoryGenerator;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Superstructure.SuperstructureStates;
import frc.lib.util.DriveSignal;

import java.util.Arrays;

public class SideStartToFrontShip extends AutoModeBase {

    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    private final boolean mStartedLeft;
    private DriveTrajectory mFrontlash1, mFrontlash2, mFrontlash3, mFrontlash4, mFrontlash5;
    //private final double mFrontlash1WaitTime;

    public SideStartToFrontShip(boolean robotStartedOnLeft) {
        mStartedLeft = robotStartedOnLeft;
        mFrontlash1 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().midStartToNearMiddleShip.get(mStartedLeft), true);
        //mFrontlash1 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().sideStartToNearMiddleShip.get(mStartedLeft), true);
        /*mFrontlash2 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().nearMiddleShipToBuffer.get(mStartedLeft));
        mFrontlash3 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().bufferToHatch2.get(mStartedLeft));
        mFrontlash4 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().hatch2ToSlot1.get(mStartedLeft));
        mFrontlash5 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().lastTry.get(mStartedLeft));

        mFrontlash1WaitTime = mTrajectoryGenerator.getTrajectorySet().sideStartToNearMiddleShip.get(mStartedLeft).getLastState().t() - 1.0;*/
    }

    @Override
    public void done() {
        Drive.getInstance().setVelocity(DriveSignal.NEUTRAL, DriveSignal.NEUTRAL);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        // 2 hatch cargo ship auto
        System.out.println("Running Frontlash Program");
        //runAction(new WaitAction(7));
        // Start drving path1: scoring the first hatch
        runAction(new ParallelAction(
                Arrays.asList(
                        mFrontlash1,
                        new SeriesAction(
                                Arrays.asList(
                                        new SetSuperstructure(false),
                                        new WaitAction(1.5), // Don't change this #
                                        new SetSuperstructure(SuperstructureStates.LOW_POSITION)
                                )
                        )
                )
        ));

        // Start driving path2: move to buffer zone
        /*runAction(mFrontlash2);

        // Start driving path3; picking the second hatch
        runAction(new SeriesAction(
                Arrays.asList(
                        mFrontlash3,
                        new TogglePeg(true)
                )
        ));

        // Scoring the 2nd hatch woohoo
        runAction(new SeriesAction(
                Arrays.asList(
                        mFrontlash4,
                        new LLDrive()
                        //new DriveTo(20,-20),
                        //new DriveTo(20,20)
                        //mFrontlash5
                        //new TogglePeg(false)
                )
        ));*/
    }
}
