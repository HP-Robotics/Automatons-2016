package org.usfirst.frc.team2823.robot;

public class EmptyAuto extends AutoMode {

	public EmptyAuto(Robot robot) {
		super(robot, 0);
	}

	@Override
	public void autoInit() {
		System.out.println("Initializing doing nothing...");

	}

	@Override
	public void autoPeriodic() {
		int doNothingTime = 1000;
		
		System.out.println("Doing nothing for " + (doNothingTime / 1000) + " second(s)...");
		try{
			Thread.sleep(doNothingTime);
		}
		catch(Exception e)
		{
			System.out.println("Fatal error!\n\nWho cares? Taylor does... Continuing...");
		}
		System.out.println("Nothing done successfully!");
	}
}
