package frc.robot.auto.actions;

import frc.lib.util.DriveSignal;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj.Timer;

public class SetOpenLoop implements Action {
    private static final Drive mDrive = Drive.getInstance();
    private double mOutput, mDuration;
    private double mStartTime;

    public SetOpenLoop(double output, double duration) {
        mOutput = output;
        mDuration = duration;
    }

    @Override
    public void start() {
        //mDrive.setBrakeMode(true);
		mStartTime = Timer.getFPGATimestamp();
    }

    @Override
    public void update() {
	    mDrive.setOpenLoop(new DriveSignal(-mOutput, mOutput));
    }

    @Override
    public boolean isFinished() {
        boolean timedOut = Timer.getFPGATimestamp() - mStartTime > mDuration;
        return timedOut;
    }

    @Override
    public void done() {
        mDrive.stop();
        //mDrive.setBrakeMode(false);
    }
}