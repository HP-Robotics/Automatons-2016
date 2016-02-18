package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;


public class AutoMode {
	Robot robot;
	Timer tick;
	int stage = 0;
	double stageTimeouts[];
	
	
	public AutoMode(Robot myBot) {
		robot = myBot;
	}
	
	public void setStageTimeouts(double[] t) {
		stageTimeouts = t;

	}
	
	public void startAuto() {
		stage = 0;
		
		tick = new Timer();
		tick.reset();
		tick.start();
	}
	
	public void autoInit() {
		
		System.out.println("Override me");
		
	};
	public void autoPeriodic() {
		System.out.println("Override me");
	}

	public boolean checkStageTimeout() {
		if (stage < 0 || stage >= stageTimeouts.length)
			return true;
	
		if (tick.get() > stageTimeouts[stage]) {

			System.out.printf("stage %d timed out\n", stage);
			nextStage();
			return true;
		}
		return false;
	}
	
	public void nextStage() {
		System.out.printf("Stage Finished: %d\tTime: %f\tTotal Time:%f\n",stage,tick.get(),DriverStation.getInstance().getMatchTime());
		tick.reset();
		stage++;
		
		if(stage >= stageTimeouts.length) {
			endAuto();
		}
	}
	
	public void endAuto() {
		robot.driveRobot(0.0, 0.0);
	}
}
