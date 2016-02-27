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
		
		//robot.armControl.setSetpoint(Robot.HIGHTRAVELSETPOINT);
		//robot.armControl.enable();
		
		//robot.gyroDriveControl.enableLog("TestGyroPID.csv");
		//robot.gyroDriveControl.enable();
		
		//robot.gyroDriveControl.setSetpoint(SmartDashboard.getNumber("TestGyro Target (Inches)"));
		
		initTime = Timer.getFPGATimestamp();
		double power = SmartDashboard.getNumber("TestDrive Power");
		robot.driveRobot(power, power);
	}
	
	public void testPeriodic() {
		//robot.gyroDriveControl.setPID(SmartDashboard.getNumber("P"), SmartDashboard.getNumber("I"), SmartDashboard.getNumber("D"));
		
		if((Timer.getFPGATimestamp() - initTime) > 2) {
			robot.driveRobot(0.0, 0.0);
		}
		
		SmartDashboard.putNumber("Left Encoder (Inches)", Robot.driveEncoderToInches(robot.lDriveEncoder.get()));
		SmartDashboard.putNumber("Right Encoder (Inches)", Robot.driveEncoderToInches(robot.rDriveEncoder.get()));
		SmartDashboard.putNumber("Arm Encoder", robot.armEncoder.get());
	}
}