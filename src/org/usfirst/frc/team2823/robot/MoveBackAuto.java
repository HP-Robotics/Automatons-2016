package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MoveBackAuto extends AutoMode {

	public MoveBackAuto(Robot myBot) {
		super(myBot);
	}

	@Override
	public void autoInit() {
		double[] t = {3.0, 1.0};
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
			driveForward();
			break;
		case 1:
			//driveForward();
			robot.driveRobot(0.0, 0.0);
			break;
		}
	}
	
	public void driveForward() {
		 robot.goNoDrifting(0.2, -robot.gyro.getAngle() * SmartDashboard.getNumber("k_angle"), 0.5, 0.5);
	     //robot.driveRobot(0.2, 0.2);
	}
	
	public void driveBack() {
		robot.goNoDrifting(-0.2, -robot.gyro.getAngle() * SmartDashboard.getNumber("k_angle"), -0.5, 0.5);
		//robot.driveRobot(-0.2, -0.2);
	}
}