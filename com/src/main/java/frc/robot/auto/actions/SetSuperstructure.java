
package frc.robot.auto.actions;

import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureStates;

public class SetSuperstructure implements Action {
    private static final Superstructure mSuperstructure = Superstructure.getInstance();
    private SuperstructureStates mState;

    public SetSuperstructure(SuperstructureStates state) {
        mState = state;
    }

    public SetSuperstructure(boolean isCargoMode) {
        mSuperstructure.setIsCargo(isCargoMode);
    }

    @Override
    public void start() {
    	mSuperstructure.setDesiredState(mState);
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