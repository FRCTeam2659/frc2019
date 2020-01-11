package frc.robot.auto.actions;

import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj.Timer;

public class DriveTurnToVisionTarget implements Action {
    private Drive mDrive;

    private Limelight mLimelight;
    private long mTimeout;
    private double mStartTime;

    public DriveTurnToVisionTarget(long timeout){
        mTimeout = timeout;
        mDrive = Drive.getInstance();
        mLimelight = Limelight.getInstance();
    }


    @Override
    public void start() {
        mStartTime = Timer.getFPGATimestamp();
    }

    @Override
    public void update() {
        if (mLimelight.getTargetExists())
            mDrive.driveTurnToAngleWithForwardVelocity(0);
    }

    @Override
    public boolean isFinished() {
        boolean timedOut = Timer.getFPGATimestamp() - mStartTime > mTimeout;
        return mDrive.IsWithinAngle() || timedOut;
    }

    @Override
    public void done() {
        mDrive.stop();
    }
}