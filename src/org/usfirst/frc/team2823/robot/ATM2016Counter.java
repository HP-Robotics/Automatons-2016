package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.Timer;

/**
 * Class for counting the number of ticks on a digital input channel. This is a
 * general purpose class for counting repetitive events. It can return the
 * number of counts, the period of the most recent cycle, and detect when the
 * signal being counted has stopped by supplying a maximum cycle time.
 *
 * All counters will immediately start counting - reset() them if you need them
 * to be zeroed before use.
 */
public class ATM2016Counter extends Counter {
	private double m_previousCount = 0;
	private double m_previousTime = Timer.getFPGATimestamp();
	
	@Override public double getRate() {
		double rate;
		double count = get();
		double time = Timer.getFPGATimestamp();
		
		rate = (count - m_previousCount) / (time - m_previousTime);
		
		m_previousCount = count;
		m_previousTime = time;
		
		return rate;
		
	}
	
	@Override public double pidGet() {
		return getRate();
	}
	
	public double getRateInRPMs() {
		return (getRate() * (60.0/2048.0));
	}
}