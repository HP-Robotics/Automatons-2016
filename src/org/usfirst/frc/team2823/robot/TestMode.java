package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TestMode {
	Robot robot;
	double initTime;
	boolean testPIDEnabled = false;
	
	public TestMode(Robot robot2) {
		robot = robot2;
	}
	
	public void testInit() {
		robot.gyroReset();
		robot.lDriveEncoder.reset();
		robot.rDriveEncoder.reset();
		
		robot.shooterSpeedControl.reset();
		
		//robot.trigger.setAngle(SmartDashboard.getNumber("Servo Angle"));
		
		//robot.armControl.setSetpoint(Robot.HIGHTRAVELSETPOINT);
		//robot.armControl.enable();
		
		/*robot.gyroDriveControl.enableLog("TestGyroPID.csv");
		
		//robot.gyroDriveControl.setSetpoint(SmartDashboard.getNumber("TestGyro Target (Inches)"));
		robot.gyroDriveControl.setOutputRange(-0.5,  0.5);
		robot.gyroDriveControl.setSetpoint(1000);
		
		robot.gyroDriveControl.enable();*/
		
		//robot.driveRobot(0.5, 0.5);
		
    	//begin driving portcullis arm into the robot
    	robot.portcullisArm.set(Robot.PORTCULLIS_LOW_POWER * Robot.PORTCULLIS_UP);
    	
		initTime = Timer.getFPGATimestamp();
	}
	
	public void testPeriodic() {
		//robot.motionDriveControl.setPID(SmartDashboard.getNumber("P"), SmartDashboard.getNumber("I"), SmartDashboard.getNumber("D"));
		robot.turnControl.setPID(SmartDashboard.getNumber("P"), SmartDashboard.getNumber("I"), SmartDashboard.getNumber("D"));
		//robot.gyroDriveControl.setPID(SmartDashboard.getNumber("P"), SmartDashboard.getNumber("I"), SmartDashboard.getNumber("D"));
		
		/*if(Timer.getFPGATimestamp() - initTime > 4) {
			robot.gyroDriveControl.disable();
			robot.gyroDriveControl.closeLog();
		}*/
		
		if(robot.stick1.getRawButton(Robot.YBUTTON)) {
			robot.trigger.setAngle(SmartDashboard.getNumber("Servo Angle"));
			System.out.println("Angle: " + robot.trigger.getAngle());
			System.out.println("Setting angle to " + SmartDashboard.getNumber("Servo Angle"));
		}
		
		if(robot.stick1.getRawButton(Robot.ABUTTON)) {
			if (!robot.motionDriveEnabled) {
				robot.lDriveEncoder.reset();
				robot.rDriveEncoder.reset();
				robot.motionDriveEnabled = true;
				robot.tankDriveEnabled = false;
				robot.slowDriveEnabled = false;

				robot.leftDriveControl.enableLog("leftControlPID.csv");
				robot.rightDriveControl.enableLog("rightControlPID.csv");
				//robot.leftDriveControl.configureGoal(SmartDashboard.getNumber("Motion Plan Target"), Robot.MAXVELOCITY/3, Robot.MAXACCELERATION/5);
				robot.leftDriveControl.configureGoal(SmartDashboard.getNumber("Motion Plan Target"), Robot.MAXVELOCITY/3, Robot.MAXACCELERATION/5);
				//robot.rightDriveControl.configureGoal(SmartDashboard.getNumber("Motion Plan Target"), Robot.MAXVELOCITY/3, Robot.MAXACCELERATION/5);
				robot.rightDriveControl.configureGoal(SmartDashboard.getNumber("Motion Plan Target")*0.95, (Robot.MAXVELOCITY/3)*(0.95), (Robot.MAXACCELERATION/5)*(0.95));
				robot.leftDriveControl.enable();
				robot.rightDriveControl.enable();

			}

		} else if(robot.motionDriveEnabled) {
			robot.motionDriveEnabled = false;
			robot.tankDriveEnabled = true;
			robot.slowDriveEnabled = false;

			robot.leftDriveControl.disable();
			robot.rightDriveControl.disable();
			robot.leftDriveControl.closeLog();
			robot.rightDriveControl.closeLog();
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
		
		if(robot.stick2.getRawButton(Robot.LTRIGGER)) {
			if(!robot.turn){
				robot.gyroReset();
				robot.turnControl.setSetpoint(SmartDashboard.getNumber("Motion Plan Target"));
				robot.turnControl.enable();
				robot.turn = true;
			}
			
		}else{
			robot.turnControl.disable();
			robot.turn = false;
		}
		
		SmartDashboard.putNumber("Gyro Rotation", robot.gyro.getAngle());
		SmartDashboard.putNumber("Left Encoder (Inches)", Robot.driveEncoderToInches(robot.lDriveEncoder.get()));
		SmartDashboard.putNumber("Right Encoder (Inches)", Robot.driveEncoderToInches(robot.rDriveEncoder.get()));
		SmartDashboard.putNumber("Arm Encoder", robot.armEncoder.get());
	}
}