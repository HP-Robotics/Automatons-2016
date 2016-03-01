package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CrossDefenseAuto extends AutoMode {
	
	public CrossDefenseAuto(Robot myBot) {
		super(myBot);
	}

	@Override
	public void autoInit() {
		double[] timeouts = {1.0, 15.0, 5.0};
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
			lowerArmBasedOnDefense();
			break;
		case 2:
			driveOverDefense();
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
	
	public void lowerArmBasedOnDefense() {
		
		//run entry code
		if(!stageData[stage].entered) {
			
			robot.armControl.enable();
			
			//determine setpoint based on SmartDashboard input
			if(!SmartDashboard.getBoolean("Lowbar?")) {
				robot.armControl.setSetpoint(Robot.MIDSETPOINT);
				robot.currentSetpoint = 2;
			} else {
				robot.armControl.setSetpoint(Robot.LOWTRAVELSETPOINT);
				robot.currentSetpoint = 3;
			}
			
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
			
			//drive with gyro motion plan over the defense
			robot.gyroDriveControl.enableLog("autoGyroDrivePID.csv");
			robot.gyroDriveControl.configureGoal(150, Robot.MAXVELOCITY/2, Robot.MAXACCELERATION/4);
			robot.gyroDriveControl.enable();
			
			stageData[stage].entered = true;
		}
		
		//move on to the next stage when the motion plan is finished
		if(robot.gyroDriveControl.isPlanFinished()) {
			nextStage();
		}
	}
}