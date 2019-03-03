
package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Intake;

public class SetIntaking implements Action {
    private static final Intake mIntake = Intake.getInstance();

    private final boolean mWaitUntilHasCube;
    private double mStartTime;

    public SetIntaking(boolean waitUntilHasCube) {
        mWaitUntilHasCube = waitUntilHasCube;
    }

    @Override
    public void start() {
        mStartTime = Timer.getFPGATimestamp();
    }

    @Override
    public void update() {
    	mIntake.intake();
    }

    @Override
    public boolean isFinished() {
        if (mWaitUntilHasCube) {
            boolean timedOut = Timer.getFPGATimestamp() - mStartTime > 2.5;
            if (timedOut) {
                System.out.println("Timed out!!!!!");
            }
            return mIntake.isCargoLoaded() || timedOut;
        } else {
            return false;
        }
    }

    @Override
    public void done() {
        mIntake.stop();
    }
}