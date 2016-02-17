package org.usfirst.frc.team2823.robot;

public class EmptyAuto implements AutoMode {

	@Override
	public void autoInit() {
		System.out.println("Initializing doing nothing...");

	}

	@Override
	public void autoPeriodic() {
		System.out.println("Doing nothing...");
		try{
			Thread.sleep(10000);
		}
		catch(Exception e)
		{
			System.out.println("Fatal error!\n\nWho cares? Continuing...");
		}
		System.out.println("Nothing done successfully!");
	}
}
