package org.usfirst.frc.team2823.robot;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

import edu.wpi.first.wpilibj.Timer;

public class CalibrateAuto implements AutoMode {
	
	Robot robot;
	
	int autoCounter;
	double autoTimePrev;
	double autoPosPrev;
	double autoVelPrev;
	double initTime;
	
	/*declare CSV file-writing-related objects and variables*/
	File f;
	BufferedWriter bw;
	FileWriter fw;
	
	public CalibrateAuto(Robot myBot) {
		robot = myBot;
	}
	
	@Override
	public void autoInit() {
    	//initialize values for capturing data during autonomous
    	autoCounter = 0;
    	initTime = Timer.getFPGATimestamp();
    	autoTimePrev = initTime;
    	autoPosPrev = robot.lDriveEncoder.get();
    	autoVelPrev = 0;
    	
    	//begin driving the robot
    	//driveRobot(0.2, 0.2);
    	
    	createFileWritingObjects();
    	
  

	}

	@Override
	public void autoPeriodic() {

    	//drive the robot for a certain number of seconds
    	if((Timer.getFPGATimestamp()-initTime) < 4.0 ) {
			
    		//every ten loops capture data to a .csv file
    		autoCounter++;
			if(autoCounter > 9) {
				double currentTime = Timer.getFPGATimestamp();
				int currentPos = robot.lDriveEncoder.get();
				
				double dT = (currentTime - autoTimePrev);
				double V = (currentPos - autoPosPrev) / dT;
				double A = (V - autoVelPrev) / dT;
				
				writeCSV("\n" + currentTime + ", " + currentPos + ", " + robot.rDriveEncoder.get() + ", " + robot.lDrive1.getSpeed() + ", " + robot.rDrive1.getSpeed() + ", " + dT + ", " + V + ", " + A, "AutoOutput");
				
				//update previous time, position, velocity
				autoTimePrev = currentTime;
				autoPosPrev = currentPos;
				autoVelPrev = V;
				autoCounter = 0;
			}
			
			
		} else {
			//stop robot once time is up
			robot.driveRobot(0.0, 0.0);
		}
	}
	  
    //function to create a file
    public void createCSV(String file) {
    	try {
    		f = new File("home/lvuser/" + file + ".csv");
    		
    		if(!f.exists()) {
    			f.createNewFile();
    		}
    		fw = new FileWriter(f);
    		
    	} catch(IOException e) {
    		e.printStackTrace();
    	}
    	
    	bw = new BufferedWriter(fw);
    	
    	try {
    		bw.write("Team2823");
    		bw.close();
    		fw.close();
    		
    	} catch(IOException e) {
    		e.printStackTrace();
    	}
    }
    //function to write data to a .csv file
    public void writeCSV(Object data, String file) {
    	try(PrintWriter csv = new PrintWriter(new BufferedWriter(new FileWriter("home/lvuser/" + file + ".csv", true)))) {
    		csv.print(", " + data.toString());
    		
    	} catch(IOException e) {
    		e.printStackTrace();
    	}
    }
    //create elements to log data to .csv files
    public void createFileWritingObjects() {
    	//create .csv files to log data
    	createCSV("AutoOutput");
    	createCSV("TeleopOutput");
    	
    	//write header to .csv files
    	writeCSV("\nTimestamp, Left Encoder, Right Encoder, Left Speed, Right Speed, dT, L-Vel, L-Accel", "TeleopOutput");
    	writeCSV("\nTimestamp, Left Encoder, Right Encoder, Left Speed, Right Speed, dT, L-Vel, L-Accel", "AutoOutput");
    	
    }
}
