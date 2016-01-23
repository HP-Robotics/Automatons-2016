package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TalonSRX;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	Joystick stick;
	Encoder lDriveEncoder;
	Encoder rDriveEncoder;
	VictorSP lDrive1;
	VictorSP lDrive2;
	VictorSP rDrive1;
	VictorSP rDrive2;
	TalonSRX shooter;
	
	File f;
	BufferedWriter bw;
	FileWriter fw;
	
	int autoLoopCounter;
	double motorSpeed = 0.0;
	boolean aButtonPressed = false;
	boolean yButtonPressed= false;
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	//create USB camera
    	CameraServer camera;
    	camera = CameraServer.getInstance();
    	camera.setQuality(50);
    	camera.startAutomaticCapture("cam0");
    	
    	
    	
    	//create objects
    	stick = new Joystick(0);
    	lDrive1 = new VictorSP(1);
    	lDrive2 = new VictorSP(2);
    	rDrive1 = new VictorSP(3);
    	rDrive2 = new VictorSP(4);
    	shooter = new TalonSRX(5);
    	
    	lDriveEncoder = new Encoder(0, 1, true, EncodingType.k4X);
    	rDriveEncoder = new Encoder(2, 3, true, EncodingType.k4X);
    	//create .csv file to log data
    	try {
    		f = new File("home/lvuser/Output.csv");
    		
    		if(!f.exists()) {
    			f.createNewFile();
    		}
    		fw = new FileWriter(f);
    		
    	} catch(IOException e) {
    		e.printStackTrace();
    	}
    	
    	bw = new BufferedWriter(fw);
    }
    
    /**
     * This function is run once each time the robot enters autonomous mode
     */
    public void autonomousInit() {
    	autoLoopCounter = 0;
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	if(autoLoopCounter < 100) //Check if we've completed 100 loops (approximately 2 seconds)
		{
			driveRobot(-0.2, -0.2); 	// drive forwards half speed
			autoLoopCounter++;
			writeCSV(autoLoopCounter);
			} else {
			lDrive1.set(0.0); 	// stop robot
		}
    }
    
    /**
     * This function is called once each time the robot enters tele-operated mode
     */
    public void teleopInit() {
    	try {
    		bw.write("Team2823");
    		bw.close();
    		fw.close();
    		
    	} catch(IOException e) {
    		e.printStackTrace();
    	}
    	writeCSV("\nTimestamp, Left Encoder, Right Encoder, Left Speed, Right Speed");
    }

    /**
     * This function is called periodically during operator control
     */
    
    public void teleopPeriodic() {
    	
    	if(stick.getRawButton(2)) {
    		if(!aButtonPressed && (motorSpeed < 1)){
    			aButtonPressed = true;
    			motorSpeed+= 0.2;
    			
    		}
    	} else {
    		aButtonPressed= false;
    	}
    	
    	
    	if(stick.getRawButton(4)) {
    		if(!yButtonPressed && (motorSpeed > -1)){
    			yButtonPressed = true;
    			motorSpeed-= 0.2;
    		}
    	}
    	else {
    		yButtonPressed= false;
    	}
    	
    	shooter.set(motorSpeed);
    	driveRobot(-stick.getRawAxis(1), - stick.getRawAxis(3));
    	writeCSV("\n" + Timer.getFPGATimestamp() + ", " + lDriveEncoder.get() + ", " + rDriveEncoder.get() + ", " + lDrive1.getSpeed() + ", " + rDrive1.getSpeed());
    	
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    	LiveWindow.run();
    }
    
    public void writeCSV(Object data) {
    	try(PrintWriter csv = new PrintWriter(new BufferedWriter(new FileWriter("home/lvuser/Output.csv", true)))) {
    		csv.print(", " + data.toString());
    		
    	} catch(IOException e) {
    		e.printStackTrace();
    	}
    }
    
    public void driveRobot(double left, double right) {
		rDrive1.set(right);
		rDrive2.set(right);
		// Values are multiplied by -1 to ensure that the motors on the right
		// spin opposite the motors on the left.
		lDrive1.set(-left);
		lDrive2.set(-left);

	}
   	
 }

