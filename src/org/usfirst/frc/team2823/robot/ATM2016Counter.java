package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.Counter;

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
	
	@Override public double getRate() {
		double rate;
		
		rate = (get() - m_previousCount) / getPeriod();
		
		return rate;
		
	}
}
