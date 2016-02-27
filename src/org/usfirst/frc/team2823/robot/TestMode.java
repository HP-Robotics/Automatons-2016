package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TestMode {
	Robot robot;
	double initTime;
	
	public TestMode(Robot newRobot) {
		robot = newRobot;
	}
	
	public void testInit() {
		robot.gyroReset();
		robot.lDriveEncoder.reset();
		robot.rDriveEncoder.reset();
		
		robot.motionDriveControl.configureGoal(50, Robot.MAXVELOCITY, Robot.MAXACCELERATION);
		//robot.armControl.setSetpoint(Robot.HIGHTRAVELSETPOINT);
		//robot.armControl.enable();
		
		//robot.gyroDriveControl.enableLog("TestGyroPID.csv");
		//robot.gyroDriveControl.enable();
		
		//robot.gyroDriveControl.setSetpoint(SmartDashboard.getNumber("TestGyro Target (Inches)"));
	}
	
	public void testPeriodic() {
		robot.motionDriveControl.setPID(SmartDashboard.getNumber("P"), SmartDashboard.getNumber("I"), SmartDashboard.getNumber("D"));
		
    	if(robot.stick.getRawButton(Robot.ABUTTON)) {
    		if (!robot.motionDriveEnabled) {
    			robot.motionDriveEnabled = true;
    			robot.tankDriveEnabled = false;
    			robot.slowDriveEnabled = false;

    			robot.motionDriveControl.enableLog("motionControlPID.csv");
    			robot.motionDriveControl.configureGoal(50, Robot.MAXVELOCITY, Robot.MAXACCELERATION);
    			robot.motionDriveControl.enable();
    		}
    		
    	} else if(robot.motionDriveEnabled) {
    		robot.motionDriveEnabled = false;
    		robot.tankDriveEnabled = true;
    		robot.slowDriveEnabled = false;
    		
    		robot.motionDriveControl.disable();
    		robot.motionDriveControl.closeLog();
    	}
		
		SmartDashboard.putNumber("Left Encoder (Inches)", Robot.driveEncoderToInches(robot.lDriveEncoder.get()));
		SmartDashboard.putNumber("Right Encoder (Inches)", Robot.driveEncoderToInches(robot.rDriveEncoder.get()));
		SmartDashboard.putNumber("Arm Encoder", robot.armEncoder.get());
	}
}