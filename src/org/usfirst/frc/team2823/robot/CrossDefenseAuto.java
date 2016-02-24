package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CrossDefenseAuto extends AutoMode {
	Robot robot;
	
	public CrossDefenseAuto(Robot myBot) {
		super(myBot);
	}

	@Override
	public void autoInit() {
		double[] timeouts = {15.0, 1.0};
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
			lowerArmBasedOnDefense();
			break;
		case 1:
			driveOverDefense();
			break;
		}
	}
	
	public void spinUpShooter() {
		//run entry code
		if(!stageData[stage].entered) {
			
			//spin shooter up to speed
			robot.shooterSpeedControl.setSetpoint(SmartDashboard.getNumber("TargetShooterSpeed"));
			robot.shooterSpeedControl.enable();
			
			stageData[stage].entered = true;
		}
	}
	
	public void lowerArmBasedOnDefense() {
		
		//run entry code
		if(!stageData[stage].entered) {
			
			robot.armControl.enable();
			
			//determine setpoint based on SmartDashboard input
			if(SmartDashboard.getString("Auto Defense").equals("LOWBAR")) {
				robot.armControl.setSetpoint(Robot.INTAKESETPOINT);
			} else {
				robot.armControl.setSetpoint(Robot.MIDSETPOINT);
			}
			
			stageData[stage].entered = true;
		}
		
		//move on to the next stage when the arm is within 100 encoder ticks
		if(Math.abs(robot.armEncoder.get() - robot.armControl.getSetpoint()) < 100) {
			nextStage();
		}
		
	}
	
	public void driveOverDefense() {
		//run entry code
		if(!stageData[stage].entered) {
			
			
			
			stageData[stage].entered = true;
		}
	}
}