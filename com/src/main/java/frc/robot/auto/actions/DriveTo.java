package frc.robot.auto.actions;

import frc.lib.util.DriveSignal;
import frc.lib.util.PIDConstants;
import frc.lib.util.PIDWrapper;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj.Timer;

public class DriveTo implements Action {
    private static final Drive mDrive = Drive.getInstance();
    private PIDWrapper LeftPID, RightPID;
    private double mBaseLeft;
    private double mBaseRight;
    private double mStartTime;

    public DriveTo(int left, int right) {
        mBaseLeft = mDrive.getLeftEncoderDistance();
        mBaseRight = mDrive.getRightEncoderDistance();
        LeftPID = new PIDWrapper(new PIDConstants(0.05, 0.0, 0.5, 2));
		LeftPID.setMaxOutput(0.9);
		LeftPID.setFinishedRange(2);
        LeftPID.setDesiredValue(mBaseLeft + left);
        RightPID = new PIDWrapper(new PIDConstants(0.05, 0.0, 0.5, 2));
		RightPID.setMaxOutput(0.9);
		RightPID.setFinishedRange(2);
		RightPID.setDesiredValue(mBaseRight + right);
    }

    @Override
    public void start() {
		mStartTime = Timer.getFPGATimestamp();
    }

    @Override
    public void update() {
        double l = LeftPID.calcPID(mDrive.getLeftEncoderDistance());
        double r = RightPID.calcPID(mDrive.getRightEncoderDistance());


	    mDrive.setOpenLoop(new DriveSignal(l, r));
    }

    @Override
    public boolean isFinished() {
        boolean timedOut = Timer.getFPGATimestamp() - mStartTime > 2.5;
        return (LeftPID.isDone() && RightPID.isDone()) || timedOut;
    }

    @Override
    public void done() {
    	mDrive.stop();
    }
}