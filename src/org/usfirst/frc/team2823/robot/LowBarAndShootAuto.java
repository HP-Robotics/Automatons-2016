package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LowBarAndShootAuto extends AutoMode {
	
	public LowBarAndShootAuto(Robot myBot) {
		super(myBot);
	}

	@Override
	public void autoInit() {
		double[] timeouts = {1.0, 15.0, 10.0, 3.0, 2.0, 2.0, 1.0, 1.0};
		setStageTimeouts(timeouts);
		robot.gyroReset();
		startAuto();
	}

	@Override
	public void autoPeriodic() {
		if (checkStageTimeout())
			return;
		
		switch (stage) {
		case 0:
			spinUpShooter();
			break;
		case 1:
			lowerArm();
			break;
		case 2:
			driveOverDefense();
			break;
		case 3:
			driveToWall();
			break;
		case 4:
			raiseArm();
			break;
		case 5:
			waitForFlywheelToSpinUp();
			break;
		case 6:
			turnOnTrigger();
			break;
		case 7:
			turnOffTrigger();
			break;
		}
	}
	
	public void spinUpShooter() {
		//run entry code
		if(!stageData[stage].entered) {
			
			//spin shooter up to speed
			robot.shooterSpeedControl.setSetpointInRPMs(SmartDashboard.getNumber("TargetShooterSpeed"));
			robot.shooterSpeedControl.enable();
			
			stageData[stage].entered = true;
		}
	}
	
	public void lowerArm() {
		
		//run entry code
		if(!stageData[stage].entered) {
			
			robot.armControl.enable();
			
			robot.armControl.setSetpoint(Robot.LOWTRAVELSETPOINT);
			robot.currentSetpoint = 3;
			
			stageData[stage].entered = true;
		}
		
		//FIXME remove this true or the robot will crash into something!!
		//move on to the next stage when the arm is within 100 encoder ticks
		if(true || (Math.abs(robot.armEncoder.get() - robot.armControl.getSetpoint()) < 100)) {
			nextStage();
		}
		
	}
	
	public void driveOverDefense() {
		//run entry code
		if(!stageData[stage].entered) {
			
			robot.lDriveEncoder.reset();
			robot.rDriveEncoder.reset();
			
			//drive with gyro motion plan most of the way to the wall
			robot.gyroDriveControl.enableLog("autoGyroDrivePID.csv");
			robot.gyroDriveControl.configureGoal(275, Robot.MAXVELOCITY/2, Robot.MAXACCELERATION/4);
			robot.gyroDriveControl.enable();
			
			stageData[stage].entered = true;
		}
		
		//move on to the next stage when the average of the drive encoders (in inches) is within 1 inch of the setpoint
		if(robot.gyroDriveControl.isPlanFinished()) {
			nextStage();
		}
	}
	
	public void driveToWall() {
		//run entry code
		if(!stageData[stage].entered) {
			//disable motion control
			robot.gyroDriveControl.disable();
			
			stageData[stage].entered = true;
		}
		
		//drive the rest of the way to the wall (relying on timeout to stop driving)
		robot.goNoDrifting(0.2, -robot.gyro.getAngle() * SmartDashboard.getNumber("k_angle"), 0.1, 0.5);
	}
	
	public void raiseArm() {
		
		//run entry code
		if(!stageData[stage].entered) {
			
			robot.armControl.enable();
			robot.armControl.setSetpoint(Robot.SHOOTSETPOINT);
			robot.currentSetpoint = 0;
			
			//keep driving the robot to keep it steady while shooting
			robot.driveRobot(0.1, 0.1);
			
			stageData[stage].entered = true;
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
		if(!stageData[stage].entered) {
			robot.trigger.setAngle(Robot.TRIGGERONPOSITION);
			TalkToPi.rawCommand("WATCH");
			
			stageData[stage].entered = true;
		}
		
	}
	
public void turnOffTrigger() {
		
		//run entry code
		if(!stageData[stage].entered) {
			robot.trigger.setAngle(Robot.TRIGGEROFFPOSITION);
			
			//end continual drive
			robot.driveRobot(0.0, 0.0);
			
			stageData[stage].entered = true;
		}
		
	}
}