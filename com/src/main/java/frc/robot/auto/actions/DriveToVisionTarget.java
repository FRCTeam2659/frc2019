package frc.robot.auto.actions;

import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureStates;
import edu.wpi.first.wpilibj.Timer;

public class DriveToVisionTarget implements Action {
    private final Drive mDrive = Drive.getInstance();

    private Limelight mLimelight = Limelight.getInstance();
    private Superstructure mSuperstructure = Superstructure.getInstance();
    private double mTimeout;
    private double mStartTime;
    private boolean autoRetract = false;
    private boolean mSetBrakeMode;

    public DriveToVisionTarget(double timeout) {
        this(timeout, false);
    }

    public DriveToVisionTarget(double timeout, boolean setBrakeMode) {
        mSetBrakeMode = setBrakeMode;
        mTimeout = timeout;
    }
    

    @Override
    public void start() {
        autoRetract = false;
        if (mSetBrakeMode)
            mDrive.setBrakeMode(true);
        mStartTime = Timer.getFPGATimestamp();
    }

    @Override
    public void update() {
        if (mLimelight.getTargetExists()) {
            mDrive.driveLimeLightXY();
            if (mSuperstructure.getState() != SuperstructureStates.MIDDLE_POSITION && !mSuperstructure.getIsCargo()) {
                if (mDrive.isWithinRange() || autoRetract) {
                    mSuperstructure.setDesiredState(SuperstructureStates.LOW_POSITION);
                    autoRetract = true;
                } else
                    mSuperstructure.setDesiredState(SuperstructureStates.VISION);
            }
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
        if (autoRetract)
            mSuperstructure.setDesiredState(SuperstructureStates.LOW_POSITION);
        mDrive.setBrakeMode(false);
        autoRetract = false;
    }
}