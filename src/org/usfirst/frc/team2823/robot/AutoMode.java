package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;


public class AutoMode {
	Robot robot;
	Timer tick;
	int stage = 0;
	StageDataElement[] stageData;
	
	public class StageDataElement {
		double timeout;
		boolean entered;
	}
	
	
	public AutoMode(Robot myBot) {
		robot = myBot;
	}
	
	public void setStageTimeouts(double[] t) {
		stageData = new StageDataElement[t.length];
		
		for(int i = 0; i < t.length; i++) {
			stageData[i] = new StageDataElement();
			
			stageData[i].timeout = t[i];
			stageData[i].entered = false;
		}
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
		if (stage < 0 || stage >= stageData.length)
			return true;
	
		if (tick.get() > stageData[stage].timeout) {

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
		
		if(stage >= stageData.length) {
			endAuto();
		}
	}
	
	public void endAuto() {
		robot.driveRobot(0.0, 0.0);
	}
}
