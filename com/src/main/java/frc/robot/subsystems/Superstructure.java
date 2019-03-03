package frc.robot.subsystems;

import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;
import frc.robot.states.SuperstructureConstants;

public class Superstructure extends Subsystem {

    static Superstructure mInstance = null;
    private Elevator mElevator = Elevator.getInstance();
    private Elbow mElbow = Elbow.getInstance();
    private Wrist mWrist = Wrist.getInstance();
    private Intake mIntake = Intake.getInstance();
    private MotorHatchGrabber mHatchIntake = MotorHatchGrabber.getInstance();

    private boolean isCargo = false;
    private SuperstructureStates currentState;

    public enum SuperstructureStates {
        STOWED,
        INTAKE,
        LOW_POSITION,
        MIDDLE_POSITION,
        SCORE_HIGH_CARGO,
        BACKWARD
    }

    public enum OuttakeStates {
        CARGO_OUTTAKE,
        HATCH_SIMPLE_OUTTAKE,
        HATCH_NORMAL_OUTTAKE,
        HATCH_STRONG_OUTTAKE
    }

    public synchronized static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }
        return mInstance;
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {

    }

    @Override
    public void stop() {

    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {

            @Override
            public void onStart(double timestamp) {

            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Superstructure.this) {
                }
            }

            @Override
            public void onStop(double timestamp) {

            }
        });
    }

    public void setIsCargo(boolean useCargoMode) {
        isCargo = useCargoMode;
    }
    public boolean getIsCargo() {
        return isCargo;
    }
    public synchronized void setDesiredState(SuperstructureStates desiredState) {
            if (desiredState == SuperstructureStates.STOWED) {
                setElevatorHeight(SuperstructureConstants.kStowedElevatorHeight);
                setElbowAngle(SuperstructureConstants.kStowedElbowAngle);
                setWristAngle(SuperstructureConstants.kStowedWristAngle);
                currentState = SuperstructureStates.STOWED;
            } else if (!getIsCargo()) {
                if (desiredState == SuperstructureStates.LOW_POSITION) {
                    setElevatorHeight(SuperstructureConstants.kElevatorLowHatchHeight);
                    setElbowAngle(SuperstructureConstants.kElbowLowHatchAngle);
                    setWristAngle(SuperstructureConstants.kWristLowHatchAngle);
                    currentState = SuperstructureStates.LOW_POSITION;
                } else if (desiredState == SuperstructureStates.MIDDLE_POSITION) {
                    setElevatorHeight(SuperstructureConstants.kElevatorMiddleHatchHeight);
                    setElbowAngle(SuperstructureConstants.kElbowMiddleHatchAngle);
                    setWristAngle(SuperstructureConstants.kWristMiddleHatchAngle);
                    currentState = SuperstructureStates.MIDDLE_POSITION;
                } else if (desiredState == SuperstructureStates.INTAKE) {
                    setElevatorHeight(SuperstructureConstants.kElevatorIntakeHatchHeight);
                    setElbowAngle(SuperstructureConstants.kElbowIntakeHatchAngle);
                    setWristAngle(SuperstructureConstants.kWristIntakeHatchAngle);
                    mHatchIntake.autoIntake();
                    currentState = SuperstructureStates.INTAKE;
                }
            } else if (getIsCargo()) {
                if (desiredState == SuperstructureStates.LOW_POSITION) {
                    setElevatorHeight(SuperstructureConstants.kElevatorLowCargoHeight);
                    setElbowAngle(SuperstructureConstants.kElbowLowCargoAngle);
                    setWristAngle(SuperstructureConstants.kWristLowCargoAngle);
                    currentState = SuperstructureStates.LOW_POSITION;
                } else if (desiredState == SuperstructureStates.MIDDLE_POSITION) {
                    setElevatorHeight(SuperstructureConstants.kElevatorMiddleCargoHeight);
                    setElbowAngle(SuperstructureConstants.kElbowMiddleCargoAngle);
                    setWristAngle(SuperstructureConstants.kWristMiddleCargoAngle);
                    currentState = SuperstructureStates.MIDDLE_POSITION;
                } else if (desiredState == SuperstructureStates.SCORE_HIGH_CARGO) {
                    setElevatorHeight(SuperstructureConstants.kElevatorHighCargoHeight);
                    setElbowAngle(SuperstructureConstants.kElbowHighCargoAngle);
                    setWristAngle(SuperstructureConstants.kWristHighCargoAngle);
                    currentState = SuperstructureStates.SCORE_HIGH_CARGO;
                } else if (desiredState == SuperstructureStates.BACKWARD) {
                    setElevatorHeight(SuperstructureConstants.kElevatorBackCargoHeight);
                    setElbowAngle(SuperstructureConstants.kElbowBackCargoAngle);
                    setWristAngle(SuperstructureConstants.kWristBackCargoAngle);
                    currentState = SuperstructureStates.BACKWARD;
                } else if (desiredState == SuperstructureStates.INTAKE) {
                    setElevatorHeight(SuperstructureConstants.kElevatorIntakeCargoHeight);
                    setElbowAngle(SuperstructureConstants.kElbowIntakeCargoAngle);
                    setWristAngle(SuperstructureConstants.kWristIntakeCargoAngle);
                    mIntake.intake();
                    currentState = SuperstructureStates.INTAKE;
                } 
            } 
        
    }

    public synchronized void setDesiredOuttakeState() {
        if (getIsCargo()) {
            if (currentState == SuperstructureStates.BACKWARD)
                mIntake.weakShoot();
            else
                mIntake.strongShoot();
        } else if (mHatchIntake.isHatchLoaded())
            mHatchIntake.autoOuttake();
        
        /*else if (currentState == SuperstructureStates.LOW_POSITION) {
            mIntake.releaseHatch();
            setElevatorJog(-5);
            setWristJogAngle(20);
            setElbowJogAngle(-20);
        } else if (currentState == SuperstructureStates.MIDDLE_POSITION) {
            mIntake.releaseHatch();
            setElevatorJog(-2);
        }*/
    }

    public synchronized void setElevatorHeight(double height) {
        mElevator.setMotionMagicPosition(height);
    }

    public synchronized void setElbowAngle(double angle) {
        mElbow.setMotionProfileAngle(angle);
    }

    public synchronized void setWristAngle(double angle) {
        mWrist.setMotionProfileAngle(angle);
    }

    public synchronized void setElevatorJog(double relative_inches) {
        mElevator.setJogElevator(relative_inches);
    }

    public synchronized void setElbowManually(double percentOutput) {
        mElbow.setOpenLoop(percentOutput);
    }

    public synchronized void setWristJog(double percentOutput) {
        mWrist.setOpenLoop(percentOutput);
    }

    public synchronized void setElbowJogAngle(double relativeAngle) {
        mElbow.setJogElbow(relativeAngle);
    }

    public synchronized void setWristJogAngle(double relativeAngle) {
        mWrist.setJogWrist(relativeAngle);
    }
}