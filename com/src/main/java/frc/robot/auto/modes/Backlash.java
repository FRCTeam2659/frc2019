package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.*;
import frc.robot.paths.TrajectoryGenerator;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Superstructure.SuperstructureStates;
import frc.lib.util.DriveSignal;

import java.util.Arrays;

public class Backlash extends AutoModeBase {

    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    private final boolean mStartedLeft;
    private DriveTrajectory mBacklash1, mBacklash2, mBacklash3;
    private final double mBacklash1WaitTime, mBacklash2WaitTime, mBacklash3WaitTime;

    public Backlash(boolean robotStartedOnLeft) {
        mStartedLeft = robotStartedOnLeft;
        mBacklash1 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().sideStartToShip1.get(mStartedLeft), true);
        mBacklash2 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().getSlot1ToCargo1.get(mStartedLeft));
        mBacklash3 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().getCargo1ToSlot2.get(mStartedLeft));

        mBacklash1WaitTime = mTrajectoryGenerator.getTrajectorySet().sideStartToShip1.get(mStartedLeft).getLastState().t() - 0.25 - 1.5 - 1.5;
        mBacklash2WaitTime = mTrajectoryGenerator.getTrajectorySet().getSlot1ToCargo1.get(mStartedLeft).getLastState().t() - 1.5 - 0.5 - 1.0;
        mBacklash3WaitTime = mTrajectoryGenerator.getTrajectorySet().getCargo1ToSlot2.get(mStartedLeft).getLastState().t() - 0.25 - 0.5 - 1.5;
    }

    @Override
    public void done() {
        Drive.getInstance().setVelocity(DriveSignal.NEUTRAL, DriveSignal.NEUTRAL);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        // two cargo auto
        System.out.println("Running Backlash Program");
        // Start drving path1: scoring the first cargo
        runAction(new ParallelAction(
                Arrays.asList(
                        mBacklash1,
                        new SeriesAction(
                                Arrays.asList(
                                        new SetSuperstructure(true),
                                        new WaitAction(1.5),
                                        new SetSuperstructure(SuperstructureStates.SCORE_HIGH_CARGO),
                                        new WaitAction(1.5),
                                        new SetSuperstructure(SuperstructureStates.BACKWARD),
                                        new WaitAction(mBacklash1WaitTime),
                                        new ShootCargo(false)
                                )
                        )
                )
        ));

        // Starrt driving path2: picking the first cargo
        runAction(new ParallelAction(
                Arrays.asList(
                        mBacklash2,
                        new SeriesAction(
                                Arrays.asList(
                                        new WaitAction(0.5),
                                        new SetSuperstructure(SuperstructureStates.SCORE_HIGH_CARGO),
                                        new WaitAction(1.5),
                                        new SetSuperstructure(SuperstructureStates.INTAKE),
                                        new WaitAction(mBacklash2WaitTime),
                                        new SetIntaking(true)
                                )
                        )
                )
        ));

        // Start driving path3: socring the second cargo
        runAction(new ParallelAction(
                Arrays.asList(
                        mBacklash3,
                        new SeriesAction(
                                Arrays.asList(
                                        new WaitAction(0.5),
                                        new SetSuperstructure(SuperstructureStates.SCORE_HIGH_CARGO),
                                        new WaitAction(1.5),
                                        new SetSuperstructure(SuperstructureStates.BACKWARD),
                                        new WaitAction(mBacklash3WaitTime),
                                        new ShootCargo(false)
                                )
                        )
                )
        ));
    }
}
