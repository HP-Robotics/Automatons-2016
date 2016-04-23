package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MagicVisionAuto extends AutoMode {

	Robot.Defense m_defenseType = Robot.Defense.OTHER;
	double m_triggerPullTime = 0.0;
	double m_visionPullTime = 0.0;

	public MagicVisionAuto(Robot robot, int defense) {
		super(robot, defense);
	}

	@Override
	public void autoInit() {
		double[] timeouts = {0.1, 0.1, 0.1, 15.0, 0.5, 15.0, 1.5, 5.0, 15.0, 0.1, 15.0};
		setStageTimeouts(timeouts);
		robot.gyroReset();

		m_defenseType = ((Robot.DefenseSelector) robot.portChevalChooser.getSelected()).getDefense();

		startAuto();
	}

	@Override
	public void autoPeriodic() {
		if (checkStageTimeout())
			return;

		switch (m_stage) {
		case 0:
			spinUpShooter();
			nextStage();
			break;
		case 1:
			if(m_defenseType == Robot.Defense.PORTCULLIS) {
				lowerPortcullisArm();
			}
			nextStage();
			break;
		case 2:
			lowerArm();
			nextStage();
			break;
		case 3:
			if(m_defenseType == Robot.Defense.CHEVAL) {
				driveToDefense();
			} else {
				nextStage();
			}
			break;
		case 4:
			if(m_defenseType == Robot.Defense.CHEVAL) {
				lowerPortcullisArm();
			} else {
				nextStage();
			}
			break;
		case 5:
			driveOverDefense();
			break;
		case 6:
			turnLeft();
			break;
		case 7:
			if(m_defense < 4){
				driveBackwards();
			}else if(m_defense > 4) {
				driveLeft();
			} else {
				nextStage();
			}
			break;
		case 8:
			shootWithVision();
			break;
		case 9:
			turnOffTrigger();
			nextStage();
			break;
		case 10:
			lowerArm();
			break;
		}
	}

	public void spinUpShooter() {
		//run entry code
		if(!stageData[m_stage].entered) {

			//spin shooter up to speed
			robot.shooterSpeedControl.setSetpointInRPMs(Robot.CLOSESPEED);
			
			robot.shooterSpeedControl.enable();

			stageData[m_stage].entered = true;
		}
	}

	public void lowerArm() {

		//run entry code
		if(!stageData[m_stage].entered) {

			robot.armControl.enable();

			if(m_defense == 1) {
				robot.armControl.setSetpoint(Robot.INTAKESETPOINT);
				robot.currentSetpoint = 4;
				
			} else if(m_defenseType == Robot.Defense.CHEVAL) {
				robot.armControl.setSetpoint(Robot.CHEVALSETPOINT);
				robot.currentSetpoint = 4;
				
			} else {
				robot.armControl.setSetpoint(Robot.MIDSETPOINT);
				robot.currentSetpoint = 2;
			}

			stageData[m_stage].entered = true;
		}
	}

	public void driveToDefense() {
		//run entry code
		if(!stageData[m_stage].entered) {

			robot.lDriveEncoder.reset();
			robot.rDriveEncoder.reset();

			int target = 41;

			//drive with gyro motion plan to the defense
			robot.gyroDriveControl.enableLog("autoGyroDrivePID.csv");

			robot.gyroDriveControl.configureGoal(target, Robot.MAXVELOCITY/2, Robot.MAXACCELERATION/4);
			robot.gyroDriveControl.enable();

			stageData[m_stage].entered = true;
		}

		//move on to the next stage when the motion plan is finished and the arm is lowered
		if(Math.abs(robot.armEncoder.get() - robot.armControl.getSetpoint()) < 100 && robot.gyroDriveControl.isPlanFinished()) {
			nextStage();
		}
	}

	public void lowerPortcullisArm() {
		if(!stageData[m_stage].entered) {

			robot.portcullisArm.set(Robot.PORTCULLIS_HIGH_POWER * Robot.PORTCULLIS_DOWN);

			stageData[m_stage].entered = true;
		}
	}

	public void driveOverDefense() {
		//run entry code
		if(!stageData[m_stage].entered) {

			robot.lDriveEncoder.reset();
			robot.rDriveEncoder.reset();

			int target = 130;

			//drive with gyro motion plan most of the way to the wall
			robot.gyroDriveControl.enableLog("autoGyroDrivePID.csv");
			
			//if the robot hasn't driven to the defense already, increase the target by 41
			if(!(m_defenseType == Robot.Defense.CHEVAL)) {
				target += 41;
			}
			
			robot.gyroDriveControl.configureGoal(target, Robot.MAXVELOCITY/1.5, Robot.MAXACCELERATION/3);

			robot.gyroDriveControl.enable();

			stageData[m_stage].entered = true;
		}

		//raise the portcullis arm once the defense is crossed
		if(Robot.driveEncoderToInches(robot.lDriveEncoder.get()) > 140 && Robot.driveEncoderToInches(robot.rDriveEncoder.get()) > 140) {
			robot.portcullisArm.set(Robot.PORTCULLIS_HIGH_POWER * Robot.PORTCULLIS_UP);
		}

		//move on to the next stage when the motion plan is finished
		if(robot.gyroDriveControl.isPlanFinished()) {
			nextStage();
		}
	}

	public void turnLeft() {
		//run entry code
		if(!stageData[m_stage].entered) {
			//disable motion control
			robot.gyroDriveControl.disable();

			//try to turn 90 degrees to the left
			robot.turnControl.enableLog("autoTurnControlLeft.csv");
			robot.turnControl.setSetpoint(-90);
			robot.turnControl.enable();

			stageData[m_stage].entered = true;
		}
		System.out.println(Math.abs(robot.gyro.getAngle() + 90));
		//continue when the robot is within 2 degrees of the target
		if(Math.abs(robot.gyro.getAngle() + 90) < 2) {
			nextStage();
		}

	}	
	public void driveLeft() {
		//run entry code
		if(!stageData[m_stage].entered) {
			//disable turn control
			robot.turnControl.disable();

			robot.lDriveEncoder.reset();
			robot.rDriveEncoder.reset();
			
			double target = (m_defense - 4) * 50.75;

			//drive with gyro motion plan most of the way to the wall
			robot.motionDriveControl.enableLog("autoDriveLeft.csv");

			robot.motionDriveControl.configureGoal(target, Robot.MAXVELOCITY/1.5, Robot.MAXACCELERATION/3);
			robot.motionDriveControl.enable();

			stageData[m_stage].entered = true;
		}

		//move on to the next stage when the motion plan is finished
		if(robot.motionDriveControl.isPlanFinished()) {
			nextStage();
		}
	}
	public void driveBackwards() {
		//run entry code
		if(!stageData[m_stage].entered) {
			//disable turn control
			robot.turnControl.disable();

			robot.lDriveEncoder.reset();
			robot.rDriveEncoder.reset();

			//drive with gyro motion plan most of the way to the wall
			robot.motionDriveControl.enableLog("autoDriveLeft.csv");
			
			robot.motionDriveControl.configureGoal(-50.75, Robot.MAXVELOCITY/1.5, Robot.MAXACCELERATION/3);
			robot.motionDriveControl.enable();

			stageData[m_stage].entered = true;
		}

		//move on to the next stage when the motion plan is finished
		if(robot.motionDriveControl.isPlanFinished()) {
			nextStage();
		}
	}

	public void shootWithVision() {
		if(!stageData[m_stage].entered) {
			
			robot.resetDrivePIDs();
			
			stageData[m_stage].entered = true;
		}
		
		if(!robot.shootingWithVision) {
			robot.shootingWithVision = true;

			//begin Pi video capture
			TalkToPi.rawCommand("WATCH6");
			robot.lastPiMessage = Timer.getFPGATimestamp();

			robot.initFrameCaptureTime = Timer.getFPGATimestamp();
			System.out.println("INIT CAPTURE TIME: " + robot.initFrameCaptureTime);
		}

		if(Math.abs(Timer.getFPGATimestamp() - robot.initFrameCaptureTime) > 0.2 && robot.shootingWithVision && !robot.capturedFirstFrame) {
			//wait for 200ms before continuing

			String piData = robot.pi.getLast();
			//System.out.println("INIT PI DATA: " + piData);
			if(piData == null){
				return;
			}

			//System.out.println("captured frame");
			
			//try to fire if the goal is visible
			if(piData.contains("GOOD")) {
				System.out.println("vision is good");
				robot.motionDriveEnabled = true;
				robot.tankDriveEnabled= false;
				robot.slowDriveEnabled = false;
				
				robot.capturedFirstFrame = true;

				//get data from Pi
				String[] piDataArray = piData.split(" ");

				robot.cameraToGoalAngle = Double.parseDouble(piDataArray[1]);
				robot.cameraToLeftEdge = Double.parseDouble(piDataArray[2]);
				robot.cameraToGoalDistance = Double.parseDouble(piDataArray[3]);
				robot.cameraToRightEdge = Double.parseDouble(piDataArray[4]);

				//pre-calculate shooter RPM based on distance to wall
				double preVisionShotSpeed = (-0.002342381 * Math.pow(robot.cameraToGoalDistance, 3)) + (0.83275224 * Math.pow(robot.cameraToGoalDistance, 2)) +
						(-89.22806 * robot.cameraToGoalDistance) + 6549.93;

				System.out.println("SHOT SPEED: " + preVisionShotSpeed);
				//spin up shooter wheel to pre-calculated RPM
				robot.shooterSpeedControl.reset();
				robot.shooterSpeedControl.setSetpointInRPMs(preVisionShotSpeed);
				robot.shooterSpeedControl.enable();

				//pre-calculate motion plan movement
				robot.preTargetPosition = (robot.cameraToLeftEdge - ((robot.cameraToLeftEdge - robot.cameraToRightEdge) / 2));
				//preTargetPosition = (cameraToGoalDistance * Math.cos(Math.toRadians(90 - cameraToGoalAngle)));

				System.out.println("PRE-TURN TARGET: " + robot.preTargetPosition);

				//check if robot is within an acceptable angle range, and set wait times accordingly
				robot.setStopTime = true;

				robot.gyroReset();

				//Turn if we need to
				if(Math.abs(robot.cameraToGoalAngle) > Robot.THRESHOLD_VISION_ANGLE) {
					robot.turnControl.setSetpoint(robot.cameraToGoalAngle);
					System.err.println("CAMERA-TO-GOAL ANGLE " + robot.cameraToGoalAngle);
					robot.turnControl.enable();

					robot.stopTime = Timer.getFPGATimestamp() + 1.3;

				}else {
					robot.stopTime = Timer.getFPGATimestamp() + 0.6;

				}
			}
		}

		//Okay, if a move is called for, do that move now.
		if(Timer.getFPGATimestamp() > (robot.stopTime - 0.6) && robot.setStopTime && !robot.clearedVisionAverage) {
			robot.clearedVisionAverage = true;

			robot.turnControl.reset();

			//pre-move based on data from before turn
			robot.lDriveEncoder.reset();
			robot.rDriveEncoder.reset();

			//double target = (preTargetPosition / Math.cos(Math.toRadians(gyro.getAngle())));
			double target = robot.preTargetPosition * Math.cos(Math.toRadians(robot.gyro.getAngle())) + robot.cameraToGoalDistance * Math.sin(Math.toRadians(robot.gyro.getAngle()));

			System.out.println("POST-TURN TURN GYRO " + robot.gyro.getAngle());
			System.out.println("POST-TURN TARGET: " + target);
			//TODO if the target is less than a threshold value, don't try to move
			robot.leftDriveControl.configureGoal(robot.shortDistanceCorrect(target + Robot.VISION_DRIVE_OFFSET), Robot.MAXVELOCITY/3, Robot.MAXACCELERATION/5);
			robot.rightDriveControl.configureGoal(robot.shortDistanceCorrect(target + Robot.VISION_DRIVE_OFFSET), Robot.MAXVELOCITY/3, Robot.MAXACCELERATION/5);
			robot.leftDriveControl.enable();
			robot.rightDriveControl.enable();


		}


		if(robot.leftDriveControl.isPlanFinished() && robot.rightDriveControl.isPlanFinished() && /*calculatedShotDistance*/ robot.clearedVisionAverage && !robot.atShotPosition) {
			robot.atShotPosition = true;
			
			//clear Pi running average
			TalkToPi.rawCommand("CLEAR");
			robot.lastPiMessage = Timer.getFPGATimestamp();
			
			//raise the arm
			robot.disableAndResetArmPid();
			robot.armControl.setSetpoint(Robot.SHOOTSETPOINT);
			robot.enableArmPid();
			
			robot.leftDriveControl.disable();
			robot.rightDriveControl.disable();
		}
		
		// If the arm is up, and speed is good, wait 200 ms for vision to stabilize
		if((robot.armEncoder.get() < (Robot.SHOOTSETPOINT + Robot.OFFSET)) && robot.shooterIsAtSpeed(100) && robot.atShotPosition && !robot.waitingToCheck) {
			robot.checkWaitTime = Timer.getFPGATimestamp();
			robot.waitingToCheck = true;
		}

		// If vision has stabilized, look again.
		if(Math.abs(Timer.getFPGATimestamp() - robot.checkWaitTime) > 1.0 && robot.waitingToCheck && !robot.checkingShotAccuracy) {
			robot.checkingShotAccuracy = true;
			String piData = robot.pi.getLast();
			if(piData.contains("GOOD")) {
				//get data from Pi
				System.out.println("CHECK PI DATA: " + piData);
				String[] piDataArray = piData.split(" ");

				robot.cameraToLeftEdge = Double.parseDouble(piDataArray[2]);
				robot.cameraToRightEdge = Double.parseDouble(piDataArray[4]);

				double target = (robot.cameraToLeftEdge - ((robot.cameraToLeftEdge - robot.cameraToRightEdge) / 2));
				System.out.println("POST-CHECK TARGET: " + target);

				if (Math.abs(target + Robot.VISION_DRIVE_OFFSET) > Robot.VISION_DRIVE_ACCURATE_ENOUGH) {
					robot.leftDriveControl.configureGoal(robot.shortDistanceCorrect(target + Robot.VISION_DRIVE_OFFSET), Robot.MAXVELOCITY/3, Robot.MAXACCELERATION/5);
					robot.rightDriveControl.configureGoal(robot.shortDistanceCorrect(target + Robot.VISION_DRIVE_OFFSET), Robot.MAXVELOCITY/3, Robot.MAXACCELERATION/5);
					robot.lDriveEncoder.reset();
					robot.rDriveEncoder.reset();
					robot.leftDriveControl.enable();
					robot.rightDriveControl.enable();
					System.out.println("POST-CHECK CORRECTION: " + (target + Robot.VISION_DRIVE_OFFSET));
				}
			} else {
				System.out.println("PI DATA NOT GOOD; FIRING ANYWAY");
			}
		}
		//wait for 200ms before continuing

		if(robot.leftDriveControl.isPlanFinished() && robot.rightDriveControl.isPlanFinished() && robot.checkingShotAccuracy && !robot.visionShotInProgress) {
			robot.visionShotInProgress = true;
			m_visionPullTime = Timer.getFPGATimestamp();
			
			
			robot.leftDriveControl.disable();
			robot.rightDriveControl.disable();
			
			System.out.println("SHOOTING!");
			System.out.println("LAST MOVE WAS " + Robot.driveEncoderToInches(robot.lDriveEncoder.get()));
			//try to shoot if the arm is above the high travel setpoint and the shooter is at speed
			robot.trigger.setAngle(Robot.TRIGGERONPOSITION);
		}
		
		if(robot.leftDriveControl.isPlanFinished() && robot.rightDriveControl.isPlanFinished() && robot.visionShotInProgress && Math.abs(Timer.getFPGATimestamp() - m_visionPullTime) > 2.0){
			robot.shootingWithVision = false;
			robot.capturedFirstFrame = false;
			robot.calculatedShotDistance = false;
			robot.clearedVisionAverage = false;
			robot.atShotPosition = false;
			robot.visionShotInProgress = false;
			robot.waitingToCheck = false;
			robot.checkingShotAccuracy = false;
			robot.motionDriveEnabled = false;
			robot.setStopTime = false;
						
			robot.shooterSpeedControl.reset();
			robot.leftDriveControl.reset();
			robot.rightDriveControl.reset();
			robot.turnControl.reset();
			
			nextStage();
		}
	}
	
	public void turnOffTrigger() {

		//run entry code
		if(!stageData[m_stage].entered) {
			robot.trigger.setAngle(Robot.TRIGGEROFFPOSITION);

			//end continual drive
			robot.driveRobot(0.0, 0.0);

			stageData[m_stage].entered = true;
		}

	}
}
