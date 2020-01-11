package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;
import frc.robot.states.SuperstructureConstants;

public class Superstructure extends Subsystem {

    static Superstructure mInstance = null;
    private Elevator mElevator = Elevator.getInstance();
    private Elbow mElbow = Elbow.getInstance();
    private Wrist mWrist = Wrist.getInstance();
    private Intake mIntake = Intake.getInstance();
    //private MotorHatchGrabber mHatchIntake = MotorHatchGrabber.getInstance();
    private Joystick mOperatorStick = new Joystick(1);

    private boolean isCargo = false;
    private SuperstructureStates currentState;

    public enum SuperstructureStates {
        STOWED,
        INTAKE,
        LOW_POSITION,
        MIDDLE_POSITION,
        SCORE_HIGH_CARGO,
        VISION
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
        if (useCargoMode) {
            mIntake.clampHatch();
            mOperatorStick.setRumble(RumbleType.kLeftRumble, 1);
            mOperatorStick.setRumble(RumbleType.kRightRumble, 1);
            Timer.delay(0.5);
            mOperatorStick.setRumble(RumbleType.kLeftRumble, 0);
            mOperatorStick.setRumble(RumbleType.kRightRumble, 0);
        }
    }
    public SuperstructureStates getState() {
        return currentState;
    }
    public boolean getIsCargo() {
        return isCargo;
    }
    public synchronized void setDesiredState(SuperstructureStates desiredState) {
            if (desiredState == SuperstructureStates.STOWED) {
                setElevatorHeight(SuperstructureConstants.kStowedElevatorHeight);
                setElbowAngle(SuperstructureConstants.kStowedElbowAngle);
                setWristAngle(SuperstructureConstants.kStowedWristAngle);
                Limelight.getInstance().useMainCameraAsVision();
                currentState = SuperstructureStates.STOWED;
            } else if (desiredState == SuperstructureStates.VISION) {
                setElevatorHeight(SuperstructureConstants.kElevatorLowHatchHeight);
                setElbowAngle(SuperstructureConstants.kElbowLowHatchAngle);
                setWristAngle(SuperstructureConstants.kWristVisionAngle);
            } else if (!getIsCargo()) {
                if (desiredState == SuperstructureStates.LOW_POSITION) {
                    setElevatorHeight(SuperstructureConstants.kElevatorLowHatchHeight);
                    setElbowAngle(SuperstructureConstants.kElbowLowHatchAngle);
                    setWristAngle(SuperstructureConstants.kWristLowHatchAngle);
                    Limelight.getInstance().useMainCameraAsVision();
                    currentState = SuperstructureStates.LOW_POSITION;
                } else if (desiredState == SuperstructureStates.MIDDLE_POSITION) {
                    setElevatorHeight(SuperstructureConstants.kElevatorMiddleHatchHeight);
                    setElbowAngle(SuperstructureConstants.kElbowMiddleHatchAngle);
                    setWristAngle(SuperstructureConstants.kWristMiddleHatchAngle);
                    Limelight.getInstance().useSecondaryCameraAsVision();
                    currentState = SuperstructureStates.MIDDLE_POSITION;
                } else if (desiredState == SuperstructureStates.INTAKE) {
                    setElevatorHeight(SuperstructureConstants.kElevatorIntakeHatchHeight);
                    setElbowAngle(SuperstructureConstants.kElbowIntakeHatchAngle);
                    setWristAngle(SuperstructureConstants.kWristIntakeHatchAngle);
                    //mHatchIntake.autoIntake();
                    mIntake.releaseHatch();
                    currentState = SuperstructureStates.INTAKE;
                }
            } else if (getIsCargo()) {
                if (desiredState == SuperstructureStates.LOW_POSITION) {
                    setElevatorHeight(SuperstructureConstants.kElevatorCargoShipHeight);
                    setElbowAngle(SuperstructureConstants.kElbowCargoShipAngle);
                    setWristAngle(SuperstructureConstants.kWristCargoShipAngle);
                    Limelight.getInstance().useMainCameraAsVision();
                    currentState = SuperstructureStates.LOW_POSITION;
                } else if (desiredState == SuperstructureStates.MIDDLE_POSITION) {
                    setElevatorHeight(SuperstructureConstants.kElevatorMiddleCargoHeight);
                    setElbowAngle(SuperstructureConstants.kElbowMiddleCargoAngle);
                    setWristAngle(SuperstructureConstants.kWristMiddleCargoAngle);
                    Limelight.getInstance().useSecondaryCameraAsVision();
                    currentState = SuperstructureStates.MIDDLE_POSITION;
                } else if (desiredState == SuperstructureStates.SCORE_HIGH_CARGO) {
                    setElevatorHeight(SuperstructureConstants.kElevatorHighCargoHeight);
                    setElbowAngle(SuperstructureConstants.kElbowHighCargoAngle);
                    setWristAngle(SuperstructureConstants.kWristHighCargoAngle);
                    currentState = SuperstructureStates.SCORE_HIGH_CARGO;
                } else if (desiredState == SuperstructureStates.INTAKE) {
                    setElevatorHeight(SuperstructureConstants.kElevatorIntakeCargoHeight);
                    setElbowAngle(SuperstructureConstants.kElbowIntakeCargoAngle);
                    setWristAngle(SuperstructureConstants.kWristIntakeCargoAngle);
                    mIntake.intake();
                    currentState = SuperstructureStates.INTAKE;
                } 
            } 
        
    }

    public synchronized void setLevel3ClimbingPosition() {
        setElevatorHeight(SuperstructureConstants.kClimbLevel3ElevatorHeight);
        setElbowAngle(SuperstructureConstants.kClimbElbowAngle);
        setWristAngle(SuperstructureConstants.kClimbWristAngle);
    }

    public synchronized void setLevel2ClimbingPosition() {
        setElevatorHeight(SuperstructureConstants.kClimbLevel2ElevatorHeight);
        setElbowAngle(SuperstructureConstants.kClimbElbowAngle); // just hold it
        setWristAngle(SuperstructureConstants.kClimbWristAngle);
    }

    public synchronized void setClimbingRestingPosition() {
        setElevatorHeight(36);
        //setElbowAngle(SuperstructureConstants.kClimbElbowAngle); // just hold it
        //setWristAngle(SuperstructureConstants.kClimbWristAngle);
    }

    public synchronized void setDesiredOuttakeState() {
        if (getIsCargo()) {
            mIntake.strongShoot();
        } else {//if (mHatchIntake.isHatchLoaded())
            //mHatchIntake.autoOuttake();
            mIntake.clampHatch();
        }
        
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

    public synchronized void setElbowClimbingAngle(double angle) {
        mElbow.setClimbingAngle(angle);
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

    public synchronized void setWristHomeAngle(int relativeAngle) {
        mWrist.setHomeAngle(relativeAngle);
    }

    public synchronized void setElbowHomeAngle(int relativeAngle) {
        mElbow.setHomeAngle(relativeAngle);
    }

    public synchronized void setWristJogAngle(double relativeAngle) {
        mWrist.setJogWrist(relativeAngle);
    }
}