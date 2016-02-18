package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MoveBackAuto extends AutoMode {

	public MoveBackAuto(Robot myBot) {
		super(myBot);
	}

	@Override
	public void autoInit() {
		double[] t = { 2.0, 1.0 };
		setStageTimeouts(t);
		robot.gyroReset();
		startAuto();
	}

	@Override
	public void autoPeriodic() {
		if (checkStageTimeout())
			return;
		
		switch (stage) {
		case 0:
			driveBack();
			break;
		case 1:
			driveForward();
			break;
		}
	}
	
	public void driveForward() {
		 robot.goNow(0.2, -robot.gyro.getAngle() * SmartDashboard.getNumber("k_angle"), 0.5, 0.5);
	     //robot.driveRobot(0.2, 0.2);
	}
	
	public void driveBack() {
		robot.goNow(-0.2, -robot.gyro.getAngle() * SmartDashboard.getNumber("k_angle"), -0.5, 0.5);
		//robot.driveRobot(-0.2, -0.2);
	}
}
