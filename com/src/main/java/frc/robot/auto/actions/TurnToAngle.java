
package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.util.DriveSignal;
import frc.lib.util.PIDConstants;
import frc.lib.util.PIDWrapper;
import frc.robot.subsystems.Drive;

public class TurnToAngle implements Action {
	private double targetAngle;
	private double eps; // Acceptable range
	private PIDWrapper turnPID;
	private double maxOutput;
	private double mTimeout, mStartTime;
	private Drive drive;

	// Declares needed variables
	public TurnToAngle(double targetAngle, double eps, long timeoutLength) {
		this(targetAngle, 1, eps, timeoutLength);
	}



	// Declares needed variables, the maxOutput and the rampRate
	public TurnToAngle(double targetAngle, double maxOutput,  double eps, long timeoutLength) {
		//super(RobotComponent.DRIVE, timeoutLength);
		this.targetAngle = targetAngle;
		drive = Drive.getInstance();
		this.maxOutput = maxOutput;
		this.eps = eps;
		mTimeout = timeoutLength;
	}


    @Override
    public void start() {
        this.turnPID = new PIDWrapper(new PIDConstants(0.028, 0.0, 0.2, eps));
		this.turnPID.setMaxOutput(0.9);
		this.turnPID.setFinishedRange(this.eps);
		this.turnPID.setDesiredValue(this.targetAngle);
		mStartTime = Timer.getFPGATimestamp();
		//drive.zeroSensors();
    }

    @Override
    public void update() {
        double x = -this.turnPID.calcPID(drive.getHeadingDegree());
		if (x > this.maxOutput) {
			x = this.maxOutput;
		} else if (x < -this.maxOutput) {
			x = -this.maxOutput;
		}

	    drive.setOpenLoop(new DriveSignal(x, -x));
    }

    @Override
    public boolean isFinished() {
		boolean timedOut = Timer.getFPGATimestamp() - mStartTime > mTimeout;
        return turnPID.isDone() || timedOut;
    }

    @Override
    public void done() {
        drive.stop();
    }
}