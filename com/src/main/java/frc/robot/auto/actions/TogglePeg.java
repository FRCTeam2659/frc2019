
package frc.robot.auto.actions;

import frc.robot.subsystems.Intake;

public class TogglePeg implements Action {
    private static final Intake mIntake = Intake.getInstance();

    private final boolean mWhetherClampHatch;

    public TogglePeg(boolean clampHatch) {
        mWhetherClampHatch = clampHatch;
    }

    @Override
    public void start() {
        if (mWhetherClampHatch)
            mIntake.clampHatch();
        else
            mIntake.releaseHatch();
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void done() {
    }
}