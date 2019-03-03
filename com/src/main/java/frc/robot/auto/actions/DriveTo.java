package frc.robot.auto.actions;

import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj.Timer;

public class DriveTo implements Action {
    private static final Drive mDrive = Drive.getInstance();

    private final int mLeft;
    private double mRight;
    private double mStartTime;

    public DriveTo(int left, int right) {
        mLeft = left;
        mRight = right;
    }

    @Override
    public void start() {
    		mStartTime = Timer.getFPGATimestamp();
    		mDrive.setPosition(mLeft, mRight);
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        boolean timedOut = Timer.getFPGATimestamp() - mStartTime > 2.5;
        return mDrive.isDoneWithPosition() || timedOut;
    }

    @Override
    public void done() {
    		mDrive.stop();
    }
}