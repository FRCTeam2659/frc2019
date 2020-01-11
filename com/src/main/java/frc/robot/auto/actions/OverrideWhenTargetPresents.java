package frc.robot.auto.actions;

import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj.Timer;

public class OverrideWhenTargetPresents implements Action {
    private final Drive mDrive = Drive.getInstance();

    private Limelight mLimelight = Limelight.getInstance();
    private double mTimeout;
    private double mStartTime;

    public OverrideWhenTargetPresents(double timeout) {
        mTimeout = timeout;
    }


    @Override
    public void start() {
        mStartTime = Timer.getFPGATimestamp();
    }

    @Override
    public void update() {
        if (mLimelight.getTargetExists()) {
            Drive.getInstance().overrideTrajectory(true);
            mDrive.driveLimeLightXY();
        }
    }

    @Override
    public boolean isFinished() {
        boolean timedOut = Timer.getFPGATimestamp() - mStartTime > mTimeout;
        return mDrive.getDriveStraightPID().isDone() || timedOut;
    }

    @Override
    public void done() {
        //mDrive.stop();
    }
}