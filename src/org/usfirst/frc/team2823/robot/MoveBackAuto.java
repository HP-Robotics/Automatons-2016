package org.usfirst.frc.team2823.robot;

public class MoveBackAuto extends AutoMode {

	public MoveBackAuto(Robot myBot) {
		super(myBot);
	}

	@Override
	public void autoInit() {
		double t[] = { 1.0, 2.0 };
		setStageTimeouts(t);
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
	
	public void driveBack() {
	     robot.driveRobot(0.2, 0.2);
	}
	
	public void driveForward() {
		robot.driveRobot(-0.2, -0.2);
	}
}
