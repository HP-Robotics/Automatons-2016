package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MainAuto extends AutoMode {
	
	public MainAuto(Robot myBot, int defense) {
		super(myBot, defense);
	}

	@Override
	public void autoInit() {
		double[] timeouts = {0.1, 0.1, 15.0, 15.0, 1.5, 3.0, 1.5, 5.0, 3.0, 2.0, 2.0, 1.0, 1.0, 15.0};
		setStageTimeouts(timeouts);
		robot.gyroReset();
		startAuto();
	}

	@Override
	public void autoPeriodic() {
		if (checkStageTimeout())
			return;
		
		switch (m_stage) {
		case 0:
			spinUpShooter();
			break;
		case 1:
			lowerArm();
			break;
		case 2:
			driveToDefense();
			break;
		case 3:
			driveOverDefense();
			break;
		case 4:
			//if the robot isn't going to shoot, stop running
			if(!SmartDashboard.getBoolean("Shoot in Auto?")) {
				m_stage = stageData.length;
				return;
			}
			
			if(m_defense > 2) {
				turnLeft();
			} else {
				nextStage();
			}
			break;
		case 5:
			if(m_defense > 2) {
				driveLeft();
			} else {
				nextStage();
			}
			break;
		case 6:
			if(m_defense > 2) {
				turnRight();
			} else {
				nextStage();
			}
			break;
		case 7:
			if(m_defense > 2) {
				finishDrivingToWall();
			} else {
				nextStage();
			}
			break;
		case 8:
			alignWithWall();
			break;
		case 9:
			raiseArm();
			break;
		case 10:
			waitForFlywheelToSpinUp();
			break;
		case 11:
			turnOnTrigger();
			break;
		case 12:
			turnOffTrigger();
			break;
		case 13:
			lowerArm();
		}
	}
	
	public void spinUpShooter() {
		//run entry code
		if(!stageData[m_stage].entered) {
			
			//spin shooter up to speed depending on defense
			if(m_defense == 1) {
				robot.shooterSpeedControl.setSetpointInRPMs(Robot.FARSPEED);
			} else {
				robot.shooterSpeedControl.setSetpointInRPMs(Robot.CLOSESPEED);
			}
			robot.shooterSpeedControl.enable();
			
			stageData[m_stage].entered = true;
		}
	}
	
	public void lowerArm() {
		
		//run entry code
		if(!stageData[m_stage].entered) {
			
			robot.armControl.enable();
			
			if(m_defense == 1) {
				robot.armControl.setSetpoint(Robot.LOWTRAVELSETPOINT);
				robot.currentSetpoint = 3;
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
			
			int target = 43;
			
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
	
	public void driveOverDefense() {
		//run entry code
		if(!stageData[m_stage].entered) {
			
			robot.lDriveEncoder.reset();
			robot.rDriveEncoder.reset();
			
			int target = 107;
			
			//drive with gyro motion plan most of the way to the wall
			robot.gyroDriveControl.enableLog("autoGyroDrivePID.csv");
			
			if(SmartDashboard.getBoolean("Shoot in Auto?") && (m_defense == 1 || m_defense == 2)) {
				target = 275;
			}
			
			//try to drive faster over non-lowbar defenses
			if(m_defense > 1) {
				robot.gyroDriveControl.configureGoal(target, Robot.MAXVELOCITY/1.5, Robot.MAXACCELERATION/3);
			} else {
				robot.gyroDriveControl.configureGoal(target, Robot.MAXVELOCITY/2, Robot.MAXACCELERATION/4);
			}
			
			robot.gyroDriveControl.enable();
			
			stageData[m_stage].entered = true;
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
			
			double target = (m_defense - 2) * 50.75;
			
			//drive with gyro motion plan most of the way to the wall
			robot.motionDriveControl.enableLog("autoDriveLeft.csv");
			
			robot.motionDriveControl.configureGoal(target, Robot.MAXVELOCITY/2, Robot.MAXACCELERATION/4);
			robot.motionDriveControl.enable();
			
			stageData[m_stage].entered = true;
		}
		
		//move on to the next stage when the motion plan is finished
		if(robot.motionDriveControl.isPlanFinished()) {
			nextStage();
		}
	}
	
	public void turnRight() {
		//run entry code
		if(!stageData[m_stage].entered) {
			//disable motion control
			robot.motionDriveControl.disable();
			
			//try to turn 90 degrees to the left
			robot.turnControl.enableLog("autoTurnControlRight.csv");
			robot.turnControl.setSetpoint(0);
			robot.turnControl.enable();
			
			stageData[m_stage].entered = true;
		}
		
		//continue when the robot is within 2 degrees of the target
		if(Math.abs(robot.gyro.getAngle()) < 2) {
			nextStage();
		}
		
	}
	
	public void finishDrivingToWall() {
		//run entry code
		if(!stageData[m_stage].entered) {
			//disable turn control
			robot.turnControl.disable();
			
			robot.lDriveEncoder.reset();
			robot.rDriveEncoder.reset();
			
			//drive with gyro motion plan most of the way to the wall
			robot.gyroDriveControl.enableLog("autoGyroDriveToWall.csv");
			
			robot.gyroDriveControl.configureGoal(125, Robot.MAXVELOCITY/2, Robot.MAXACCELERATION/4);
			robot.gyroDriveControl.enable();
			
			stageData[m_stage].entered = true;
		}
		
		//move on to the next stage when the motion plan is finished
		if(robot.gyroDriveControl.isPlanFinished()) {
			nextStage();
		}
	}
	
	public void alignWithWall() {
		//run entry code
		if(!stageData[m_stage].entered) {
			//disable motion control
			robot.gyroDriveControl.disable();
			
			stageData[m_stage].entered = true;
		}
		
		//drive the rest of the way to the wall (relying on timeout to stop driving)
		robot.goNoDrifting(0.2, -robot.gyro.getAngle() * SmartDashboard.getNumber("k_angle"), 0.1, 0.5);
	}
	
	public void raiseArm() {
		
		//run entry code
		if(!stageData[m_stage].entered) {
			
			robot.armControl.enable();
			robot.armControl.setSetpoint(Robot.SHOOTSETPOINT);
			robot.currentSetpoint = 0;
			
			//keep driving the robot to keep it steady while shooting
			robot.driveRobot(0.1, 0.1);
			
			stageData[m_stage].entered = true;
		}
		
		//move on to the next stage when the arm is within 100 encoder ticks
		if(Math.abs(robot.armEncoder.get() - robot.armControl.getSetpoint()) < 3) {
			nextStage();
		}
		
	}
	
	public void waitForFlywheelToSpinUp() {
		
		//move on to the next stage when the flywheel is up to speed
		if(robot.shooterIsAtSpeed(150)) {
			nextStage();
		}
	}
	
	public void turnOnTrigger() {
		
		//run entry code
		if(!stageData[m_stage].entered) {
			robot.trigger.setAngle(Robot.TRIGGERONPOSITION);
			TalkToPi.rawCommand("WATCH");
			
			stageData[m_stage].entered = true;
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