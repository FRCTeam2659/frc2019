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
    private DriveTrajectory mYolo1, mYolo2, mYolo3, mYolo4;
    private final double mYolo1WaitTime, mYolo3WaitTime, mYolo4WaitTime;

    public Yolo(boolean robotStartedOnLeft) {
        mStartedLeft = robotStartedOnLeft;
        mYolo1 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().sideStartToNearMiddleShip.get(mStartedLeft), true);
        mYolo2 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().nearMiddleShipToBuffer.get(mStartedLeft));
        mYolo3 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().bufferToCargo1.get(mStartedLeft));
        mYolo4 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().cargo1ToSlot1.get(mStartedLeft));

        mYolo1WaitTime = mTrajectoryGenerator.getTrajectorySet().sideStartToNearMiddleShip.get(mStartedLeft).getLastState().t() - 1.5;
        mYolo3WaitTime = mTrajectoryGenerator.getTrajectorySet().bufferToCargo1.get(mStartedLeft).getLastState().t() - 2.0;
        mYolo4WaitTime = mTrajectoryGenerator.getTrajectorySet().cargo1ToSlot1.get(mStartedLeft).getLastState().t() - 1.5;
    }

    @Override
    public void done() {
        Drive.getInstance().setVelocity(DriveSignal.NEUTRAL, DriveSignal.NEUTRAL);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        // 1 hatch + 1 cargo auto
        System.out.println("Running Yolo Program");
        // Start drving path1: scoring the first hatch
        runAction(new ParallelAction(
                Arrays.asList(
                        mYolo1,
                        new SeriesAction(
                                Arrays.asList(
                                        new SetSuperstructure(false),
                                        new WaitAction(1.5),
                                        new SetSuperstructure(SuperstructureStates.LOW_POSITION),
                                        new WaitAction(mYolo1WaitTime),
                                        new TogglePeg(false)
                                )
                        )
                )
        ));

        // Starrt driving path2: move to buffer zone
        runAction(new ParallelAction(
                Arrays.asList(
                        mYolo2,
                        new SeriesAction(
                                Arrays.asList(
                                        new SetSuperstructure(true),
                                        new SetSuperstructure(SuperstructureStates.INTAKE)
                                )
                        )
                )
        ));

        // Start driving path3; picking the first cargo
        runAction(new ParallelAction(
                Arrays.asList(
                        mYolo3,
                        new SeriesAction(
                                Arrays.asList(
                                        new WaitAction(mYolo3WaitTime),
                                        new SetIntaking(true)
                                )
                        )
                )
        ));
        runAction(new ParallelAction(
                Arrays.asList(
                        mYolo4,
                        new SeriesAction(
                                Arrays.asList(
                                        new SetSuperstructure(SuperstructureStates.SCORE_HIGH_CARGO),
                                        new WaitAction(1.5),
                                        new SetSuperstructure(SuperstructureStates.BACKWARD),
                                        new WaitAction(mYolo4WaitTime),
                                        new ShootCargo(false)
                                )
                        )
                )
        ));
    }
}
