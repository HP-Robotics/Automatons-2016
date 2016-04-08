package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MainAuto extends AutoMode {
	
	Robot.Defense m_defenseType = Robot.Defense.OTHER;
	
	public MainAuto(Robot robot, int defense) {
		super(robot, defense);
	}

	@Override
	public void autoInit() {
		double[] timeouts = {0.1, 0.1, 0.1, 15.0, 0.5, 15.0, 1.5, 3.0, 1.5, 5.0, 1.7, 2.0, 0.1, 1.0, 0.1, 15.0};
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
			if(m_defense == 1 || m_defenseType == Robot.Defense.CHEVAL) {
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
		case 7:
			if(m_defense > 2) {
				driveLeft();
			} else {
				nextStage();
			}
			break;
		case 8:
			if(m_defense > 2) {
				turnRight();
			} else {
				nextStage();
			}
			break;
		case 9:
			if(m_defense > 2) {
				finishDrivingToWall();
			} else {
				nextStage();
			}
			break;
		case 10:
			alignWithWall();
			break;
		case 11:
			if(!(m_defenseType == Robot.Defense.CHEVAL)) {
				raiseArm();
			} else {
				nextStage();
			}
			break;
		case 12:
			waitForFlywheelToSpinUp();
			break;
		case 13:
			turnOnTrigger();
			break;
		case 14:
			turnOffTrigger();
			nextStage();
			break;
		case 15:
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
				robot.armControl.setSetpoint(Robot.INTAKESETPOINT);
				robot.currentSetpoint = 4;
				
			} else if(m_defenseType == Robot.Defense.CHEVAL) {
				robot.armControl.setSetpoint(Robot.CHEVALSETPOINT);
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
			
			int target = 149;
			
			//drive with gyro motion plan most of the way to the wall
			robot.gyroDriveControl.enableLog("autoGyroDrivePID.csv");
			
			if(SmartDashboard.getBoolean("Shoot in Auto?") && (m_defense == 1 || m_defense == 2)) {
				target = 275;
			}
			
			//if the robot hasn't driven to the defense already, increase the target by 41
			if(!(m_defense == 1 || m_defenseType == Robot.Defense.CHEVAL)) {
				target += 41;
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
			
			robot.motionDriveControl.configureGoal(target, Robot.MAXVELOCITY/1.5, Robot.MAXACCELERATION/3);
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
			
			robot.gyroDriveControl.configureGoal(85, Robot.MAXVELOCITY/1.5, Robot.MAXACCELERATION/3);
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
			
			//lower the portcullis arm power to prevent motor damage
			robot.portcullisArm.set(Robot.PORTCULLIS_LOW_POWER * Robot.PORTCULLIS_UP);
			
			//if the robot has driven over the cheval de frise, raise the arm immediately
			if(m_defenseType == Robot.Defense.CHEVAL) {
				raiseArm();
			}
			
			stageData[m_stage].entered = true;
		}
		
		//drive the rest of the way to the wall (relying on timeout to stop driving)
		robot.goNoDrifting(0.25, -robot.gyro.getAngle() * Robot.k_ANGLE, 0.1, 0.5);
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
		
		//move on to the next stage when the arm is within 3 encoder ticks
		if(robot.armEncoder.get() < 3) {
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