package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;


public class AutoMode {
	Robot robot;
	Timer m_tick;
	double m_initTime = Timer.getFPGATimestamp();
	int m_stage = 0;
	int m_defense = 0;
	
	StageDataElement[] stageData;
	
	public class StageDataElement {
		double timeout;
		boolean entered;
	}
	
	
	public AutoMode(Robot robot2, int defense) {
		robot = robot2;
		m_defense = defense;
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
		m_stage = 0;
		
		m_tick = new Timer();
		m_tick.reset();
		m_tick.start();
	}
	
	public void autoInit() {
		
		System.out.println("Override me");
		
	};
	public void autoPeriodic() {
		System.out.println("Override me");
	}

	public boolean checkStageTimeout() {
		if (m_stage < 0 || m_stage >= stageData.length)
			return true;
	
		if (m_tick.get() > stageData[m_stage].timeout) {

			System.out.printf("stage %d timed out\n", m_stage);
			nextStage();
			return true;
		}
		return false;
	}
	
	public void nextStage() {
		System.out.printf("Stage Finished: %d\tTime: %f\tTotal Time:%f\n",m_stage,m_tick.get(),Math.abs(Timer.getFPGATimestamp() - m_initTime) - m_initTime);
		m_tick.reset();
		m_stage++;
		
		if(m_stage >= stageData.length) {
			endAuto();
		}
	}
	
	public void endAuto() {
		robot.driveRobot(0.0, 0.0);
		
		robot.gyroDriveControl.disable();
	}
}
