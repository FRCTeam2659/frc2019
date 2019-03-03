
package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Intake;

public class ShootCargo implements Action {
    private static final Intake mIntake = Intake.getInstance();
    private static final double kShootTime = 0.5;

    private double mStartTime;
    private double mPower;

    public ShootCargo(double power) {
        mPower = power;
    }
    
    public ShootCargo(boolean strongShoot) {
        if (strongShoot) 
            mPower = -1.0;
        else
            mPower = -0.6;
    }

    @Override
    public void start() {
        mStartTime = Timer.getFPGATimestamp();
        mIntake.setPower(mPower);
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - mStartTime > kShootTime;
    }

    @Override
    public void done() {
        mIntake.stop();
    }
}