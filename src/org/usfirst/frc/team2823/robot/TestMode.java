package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TestMode {
	Robot robot;
	
	public TestMode(Robot newRobot) {
		robot = newRobot;
	}
	
	public void testInit() {
		robot.gyroReset();
		robot.lDriveEncoder.reset();
		robot.rDriveEncoder.reset();
		
		robot.gyroDriveControl.enableLog("TestGyroPID.csv");
		robot.gyroDriveControl.enable();
		
		robot.gyroDriveControl.setSetpoint(Robot.driveInchesToEncoder(SmartDashboard.getNumber("TestGyro Target (Inches)")));

	}
	
	public void testPeriodic() {
		SmartDashboard.putNumber("Left Encoder (Inches)", Robot.driveEncoderToInches(robot.lDriveEncoder.get()));
		SmartDashboard.putNumber("Right Encoder (Inches)", Robot.driveEncoderToInches(robot.rDriveEncoder.get()));
	}
}