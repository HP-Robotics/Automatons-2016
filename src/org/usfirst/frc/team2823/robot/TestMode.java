package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TestMode {
	Robot robot;
	double initTime;
	boolean testPIDEnabled = false;
	
	public TestMode(Robot newRobot) {
		robot = newRobot;
	}
	
	public void testInit() {
		robot.gyroReset();
		robot.lDriveEncoder.reset();
		robot.rDriveEncoder.reset();
		
		//robot.armControl.setSetpoint(Robot.HIGHTRAVELSETPOINT);
		//robot.armControl.enable();
		
		/*robot.gyroDriveControl.enableLog("TestGyroPID.csv");
		
		//robot.gyroDriveControl.setSetpoint(SmartDashboard.getNumber("TestGyro Target (Inches)"));
		robot.gyroDriveControl.setOutputRange(-0.5,  0.5);
		robot.gyroDriveControl.setSetpoint(1000);
		
		robot.gyroDriveControl.enable();*/
		
		//robot.driveRobot(0.5, 0.5);
		initTime = Timer.getFPGATimestamp();
	}
	
	public void testPeriodic() {
		robot.motionDriveControl.setPID(SmartDashboard.getNumber("P"), SmartDashboard.getNumber("I"), SmartDashboard.getNumber("D"));
		//robot.gyroDriveControl.setPID(SmartDashboard.getNumber("P"), SmartDashboard.getNumber("I"), SmartDashboard.getNumber("D"));
		
		/*if(Timer.getFPGATimestamp() - initTime > 4) {
			robot.gyroDriveControl.disable();
			robot.gyroDriveControl.closeLog();
		}*/
		
		if(robot.stick1.getRawButton(Robot.ABUTTON)) {
			if (!robot.motionDriveEnabled) {
				robot.lDriveEncoder.reset();
				robot.rDriveEncoder.reset();
				robot.motionDriveEnabled = true;
				robot.tankDriveEnabled = false;
				robot.slowDriveEnabled = false;

				robot.motionDriveControl.enableLog("motionControlPID.csv");
				robot.motionDriveControl.configureGoal(SmartDashboard.getNumber("Motion Plan Target"), Robot.MAXVELOCITY/3, Robot.MAXACCELERATION/5);
				robot.motionDriveControl.enable();
			}

		} else if(robot.motionDriveEnabled) {
			robot.motionDriveEnabled = false;
			robot.tankDriveEnabled = true;
			robot.slowDriveEnabled = false;

			robot.motionDriveControl.disable();
			robot.motionDriveControl.closeLog();
		}
    	
		if(robot.stick1.getRawButton(Robot.BBUTTON)){
			if(!testPIDEnabled) {
				robot.lDriveEncoder.reset();
				robot.rDriveEncoder.reset();
				
				robot.motionDriveControl.enableLog("motionControlPID.csv");
				robot.motionDriveControl.setSetpoint(100);
				robot.motionDriveControl.setOutputRange(-0.5, 0.5);
				robot.motionDriveControl.enable();
				System.out.println("Start PID");

				testPIDEnabled = true;
			}
		} else if(testPIDEnabled) {
			robot.motionDriveControl.disable();
			System.out.println("Stop PID");
			robot.motionDriveControl.closeLog();
			System.out.println(robot.motionDriveControl.isEnabled());
			System.out.println(robot.motionDriveControl.getP());
			
			robot.lDriveEncoder.reset();
			robot.rDriveEncoder.reset();

			testPIDEnabled = false;
		}
		
		if(robot.stick1.getRawButton(Robot.XBUTTON)) {
			if (!robot.gyroDrive) {
				robot.gyroReset();
				robot.lDriveEncoder.reset();
				robot.rDriveEncoder.reset();
				robot.gyroDrive = true;
				robot.tankDriveEnabled = false;
				robot.slowDriveEnabled = false;

				robot.gyroDriveControl.enableLog("motionControlPID.csv");
				robot.gyroDriveControl.configureGoal(SmartDashboard.getNumber("Motion Plan Target"), Robot.MAXVELOCITY/3, Robot.MAXACCELERATION/5);
				robot.gyroDriveControl.enable();
			}

		} else if(robot.gyroDrive) {
			robot.gyroDrive = false;
			robot.tankDriveEnabled = true;
			robot.slowDriveEnabled = false;

			robot.gyroDriveControl.disable();
			robot.gyroDriveControl.closeLog();
		}
		
		
		SmartDashboard.putNumber("Left Encoder (Inches)", Robot.driveEncoderToInches(robot.lDriveEncoder.get()));
		SmartDashboard.putNumber("Right Encoder (Inches)", Robot.driveEncoderToInches(robot.rDriveEncoder.get()));
		SmartDashboard.putNumber("Arm Encoder", robot.armEncoder.get());
	}
}