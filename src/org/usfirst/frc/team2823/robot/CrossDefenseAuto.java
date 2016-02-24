package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CrossDefenseAuto extends AutoMode {
	
	public CrossDefenseAuto(Robot myBot) {
		super(myBot);
	}

	@Override
	public void autoInit() {
		//FIXME disable PID controllers at startup!!!
		double[] timeouts = {1.0, 15.0, 4.0};
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
			} else {
				robot.armControl.setSetpoint(Robot.INTAKESETPOINT);
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
			
			//drive with gyro PID to 50 inches
			robot.gyroDriveControl.enableLog("autoGyroDrivePID.csv");
			robot.gyroDriveControl.enable();
			robot.gyroDriveControl.setSetpoint(50);
			
			stageData[stage].entered = true;
		}
		
		//move on to the next stage when the average of the drive encoders (in inches) is within 1 inch of the setpoint
		if(Math.abs((Robot.driveEncoderToInches(robot.lDriveEncoder.get() + robot.rDriveEncoder.get()) / 2) - robot.gyroDriveControl.getSetpoint()) < 1) {
			nextStage();
		}
	}
}