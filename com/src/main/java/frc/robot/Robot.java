/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Arrays;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.geometry.Pose2d;
import frc.lib.util.CrashTracker;
import frc.lib.util.DriveSignal;
import frc.lib.util.TortoDriveHelper;
import frc.robot.auto.AutoModeExecutor;
import frc.robot.auto.modes.Auteleop;
import frc.robot.auto.modes.Backlash;
import frc.robot.auto.modes.CrossAutoLineMode;
import frc.robot.auto.modes.Frontlash;
import frc.robot.auto.modes.Yolo;
import frc.robot.loops.Looper;
import frc.robot.paths.TrajectoryGenerator;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.RobotStateEstimator;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Superstructure.SuperstructureStates;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends IterativeRobot {
    private Looper mEnabledLooper = new Looper();
    private Looper mDisabledLooper = new Looper();
    private TortoDriveHelper mTortoDriveHelper = new TortoDriveHelper();
    private TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();

    private final SubsystemManager mSubsystemManager = new SubsystemManager(
            Arrays.asList(
                    RobotStateEstimator.getInstance(),
                    Vision.getInstance(),
                    Drive.getInstance(),
                    Elevator.getInstance(),
                    Elbow.getInstance(),
                    Wrist.getInstance(),
                    Intake.getInstance(),
                    Superstructure.getInstance()
            )
    );

    private Drive mDrive = Drive.getInstance();
    private Vision mVision = Vision.getInstance();
    private Elevator mElevator = Elevator.getInstance();
    private Elbow mElbow = Elbow.getInstance();
    private Wrist mWrist = Wrist.getInstance();
    private Intake mIntake = Intake.getInstance();
    private Superstructure mSuperstructure = Superstructure.getInstance();
    private Joystick mDriveStick = new Joystick(0);
    private Joystick mOperatorStick = new Joystick(1);
    
    private AutoModeExecutor mAutoModeExecutor;
    private SendableChooser mStartPositionChooser, mAutoModeChooser;
    private boolean startOnLeft;
    private DriveControlState mDriveControlState = DriveControlState.OPEN_LOOP;
    private boolean isTriggered = false;
    private boolean isHatchTriggered = false;

    public Robot() {
        CrashTracker.logRobotConstruction();
    }

    enum StartingPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    enum AutoMode {
        YOLO,
        FRONTLASH,
        BACKLASH
    }

    enum DriveControlState {
        OPEN_LOOP,
        AUTELEOP
    }

    @Override
    public void robotInit() {
        try {
            mStartPositionChooser = new SendableChooser<>();
            mStartPositionChooser.addDefault("Right", StartingPosition.RIGHT);
            mStartPositionChooser.addObject("Center", StartingPosition.CENTER);
            mStartPositionChooser.addObject("Left", StartingPosition.LEFT);
            SmartDashboard.putData("Starting Position", mStartPositionChooser);
            mAutoModeChooser = new SendableChooser<>();
            mAutoModeChooser.addObject("Yolo 1 hatch + 1 cargo", AutoMode.YOLO);
            mAutoModeChooser.addObject("Frontlash 2 hatches", AutoMode.FRONTLASH);
            mAutoModeChooser.addObject("Backlash 2 cargos", AutoMode.BACKLASH);
            SmartDashboard.putData("Auto Mode", mAutoModeChooser);
            mAutoModeExecutor = new AutoModeExecutor();

            CrashTracker.logRobotInit();

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mSubsystemManager.registerDisabledLoops(mDisabledLooper);
            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());
            mDrive.setBrakeMode(false);
            mTrajectoryGenerator.generateTrajectories();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledInit() {
        SmartDashboard.putString("Match Cycle", "DISABLED");

        try {
            CrashTracker.logDisabledInit();
            mEnabledLooper.stop();
            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }

            //Drive.getInstance().zeroSensors();
            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());

            mDisabledLooper.start();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousInit() {
        SmartDashboard.putString("Match Cycle", "AUTONOMOUS");

        try {
            CrashTracker.logAutoInit();
            mDisabledLooper.stop();
            
            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());

            Drive.getInstance().zeroSensors();
            if (mStartPositionChooser.getSelected() == StartingPosition.RIGHT)
                startOnLeft = false;
            else if (mStartPositionChooser.getSelected() == StartingPosition.LEFT)
                startOnLeft = true;
            else if (mStartPositionChooser.getSelected() == StartingPosition.CENTER)
                mAutoModeExecutor.setAutoMode(new CrossAutoLineMode());
            else
                mAutoModeExecutor.setAutoMode(new CrossAutoLineMode());

            if (mAutoModeChooser.getSelected() == AutoMode.YOLO)
                mAutoModeExecutor.setAutoMode(new Yolo(startOnLeft));
            else if (mAutoModeChooser.getSelected() == AutoMode.FRONTLASH)
                mAutoModeExecutor.setAutoMode(new Frontlash(startOnLeft));
            else if (mAutoModeChooser.getSelected() == AutoMode.BACKLASH)
                mAutoModeExecutor.setAutoMode(new Backlash(startOnLeft));
            else
                mAutoModeExecutor.setAutoMode(new CrossAutoLineMode());

            mAutoModeExecutor.start();
            mEnabledLooper.start();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void teleopInit() {
        SmartDashboard.putString("Match Cycle", "TELEOP");

        try {
            CrashTracker.logTeleopInit();
            mDisabledLooper.stop();
            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }

            //RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());
            mEnabledLooper.start();

            mDrive.setVelocity(DriveSignal.NEUTRAL, DriveSignal.NEUTRAL);
            mDrive.setOpenLoop(new DriveSignal(0.05, 0.05));
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testInit() {
        SmartDashboard.putString("Match Cycle", "TEST");

        try {
            System.out.println("Starting check systems.");

            mDisabledLooper.stop();
            mEnabledLooper.stop();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledPeriodic() {
        SmartDashboard.putString("Match Cycle", "DISABLED");

        try {
            outputToSmartDashboard();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousPeriodic() {
        SmartDashboard.putString("Match Cycle", "AUTONOMOUS");

        outputToSmartDashboard();
        try {
            
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void teleopPeriodic() {
        SmartDashboard.putString("Match Cycle", "TELEOP");

        try {
            if (mDriveStick.getRawButton(1) || false) {
                if (mDriveControlState != DriveControlState.AUTELEOP) {
                    mAutoModeExecutor.setAutoMode(new Auteleop());
                    mAutoModeExecutor.start();
                    mDriveControlState = DriveControlState.AUTELEOP;
                }
            } else if (mDriveControlState == DriveControlState.OPEN_LOOP || mDriveStick.getRawButton(2)) {
                if (mDriveControlState == DriveControlState.AUTELEOP) {
                    mDriveControlState = DriveControlState.OPEN_LOOP;
                    mAutoModeExecutor.stop();
                }
                double throttle = mDriveStick.getRawAxis(1);
                double turn = mDriveStick.getRawAxis(2);
                mDrive.setOpenLoop(mTortoDriveHelper.tortoDrive(-throttle, -turn, true, true));
            }
            //mElevator.setOpenLoop(mOperatorStick.getRawAxis(1));
            if (mOperatorStick.getRawButton(1))
                mSuperstructure.setDesiredState(SuperstructureStates.LOW_POSITION);
            else if (mOperatorStick.getRawButton(2))
                mSuperstructure.setDesiredState(SuperstructureStates.STOWED);
            else if (mOperatorStick.getRawButton(3))
                mSuperstructure.setDesiredState(SuperstructureStates.MIDDLE_POSITION);
            else if (mOperatorStick.getRawButton(4))
                mSuperstructure.setDesiredState(SuperstructureStates.SCORE_HIGH_CARGO);
            else if (mOperatorStick.getRawButton(6))
                mSuperstructure.setDesiredState(SuperstructureStates.BACKWARD);
            else if(mOperatorStick.getRawButton(8))
                mSuperstructure.setDesiredState(SuperstructureStates.INTAKE);
            else if (mOperatorStick.getRawButton(7))
                mSuperstructure.setDesiredOuttakeState(); 
            else
                mIntake.stop();
            
             
            //if (mOperatorStick.getRawButton(6))
              //  mIntake.clampHatch();
            if (mOperatorStick.getRawButton(5)) {
                mIntake.releaseHatch();
                isHatchTriggered = true;
            } else if (isHatchTriggered) {
                isHatchTriggered = false;
                mIntake.triggerHatch();
            }
            if (mOperatorStick.getRawButton(10)) {
                isTriggered = true;
            } else if (isTriggered) {
                isTriggered = false;
                mSuperstructure.setIsCargo(!mSuperstructure.getIsCargo());
            }
            
            if (mDriveStick.getRawButton(8))
                mDrive.setHighGear(true);
            else if (mDriveStick.getRawButton(7))
                mDrive.setHighGear(false);
            //mVision.getCoordinate();
            outputToSmartDashboard();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testPeriodic() {
        SmartDashboard.putString("Match Cycle", "TEST");
    }

    public void outputToSmartDashboard() {
        RobotState.getInstance().outputToSmartDashboard();
        Drive.getInstance().outputTelemetry();
        Vision.getInstance().outputTelemetry();
        Elevator.getInstance().outputTelemetry();
        Elbow.getInstance().outputTelemetry();
        Wrist.getInstance().outputTelemetry();
        Intake.getInstance().outputTelemetry();
        mEnabledLooper.outputToSmartDashboard();
        // SmartDashboard.updateValues();
    }
}
