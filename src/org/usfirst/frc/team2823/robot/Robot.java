package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.TalonSRX;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.ADXL362;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;

public class Robot extends IterativeRobot {
	//create objects
	Joystick stick;
	Encoder lDriveEncoder;
	Encoder rDriveEncoder;
	VictorSP lDrive1;
	VictorSP lDrive2;
	VictorSP rDrive1;
	VictorSP rDrive2;
	TalonSRX shooter;
	ADXRS450_Gyro gyro;
	ADXL362 accelerometer;
	
	//Preferences prefs;
	
	File f;
	BufferedWriter bw;
	FileWriter fw;
	
	//declare class-wide variables
	int autoCounter;
	double autoTimePrev;
	double autoPosPrev;
	double autoVelPrev;
	double initTime;
	
	double motorSpeed = 0.0;
	boolean aButtonPressed = false;
	boolean yButtonPressed= false;
	
    public void robotInit() {
    	
    	//create USB camera
    	CameraServer camera;
    	camera = CameraServer.getInstance();
    	camera.setQuality(50);
    	camera.startAutomaticCapture("cam0");
    	
    	//create objects
    	stick = new Joystick(0);
    	lDrive1 = new VictorSP(0);
    	lDrive2 = new VictorSP(1);
    	rDrive1 = new VictorSP(2);
    	rDrive2 = new VictorSP(3);
    	shooter = new TalonSRX(4);
    	
    	lDriveEncoder = new Encoder(0, 1, true, EncodingType.k4X);
    	rDriveEncoder = new Encoder(2, 3, true, EncodingType.k4X);
    	
    	SmartDashboard.putNumber("InputSpeed", 0.0);
    	
    	//create and calibrate gyro and accelerometer
    	gyro = new ADXRS450_Gyro(Port.kOnboardCS0);
    	accelerometer = new ADXL362(ADXL362.Range.k2G);
    	
    	//gyro.calibrate();
    	
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
    	
    	try {
    		bw.write("Team2823");
    		bw.close();
    		fw.close();
    		
    	} catch(IOException e) {
    		e.printStackTrace();
    	}
    	writeCSV("\nTimestamp, Left Encoder, Right Encoder, Left Speed, Right Speed, dT, L-Vel, L-Accel");
    	//motorSpeed = prefs.getDouble("Speed", 0.0);
    }
    
    public void autonomousInit() {
    	//initialize values for capturing data during autonomous
    	autoCounter = 0;
    	initTime = Timer.getFPGATimestamp();
    	autoTimePrev = initTime;
    	autoPosPrev = lDriveEncoder.get();
    	autoVelPrev = 0;
    	
    	//begin driving the robot
    	driveRobot(0.4, 0.4);
    	
    }

    public void autonomousPeriodic() {
    	
    	//drive the robot for a certain number of seconds
    	if((Timer.getFPGATimestamp()-initTime) < 5.0 ) {
			
    		//every ten loops capture data to a .csv file
    		autoCounter++;
			if(autoCounter > 9) {
				double currentTime = Timer.getFPGATimestamp();
				int currentPos = lDriveEncoder.get();
				
				double dT = (currentTime - autoTimePrev);
				double V = (currentPos - autoPosPrev) / dT;
				double A = (V - autoVelPrev) / dT;
				
				writeCSV("\n" + currentTime + ", " + currentPos + ", " + rDriveEncoder.get() + ", " + lDrive1.getSpeed() + ", " + rDrive1.getSpeed() + ", " + dT + ", " + V + ", " + A);
				
				//update previous time, position, velocity
				autoTimePrev = currentTime;
				autoPosPrev = currentPos;
				autoVelPrev = V;
				autoCounter = 0;
			}
			
			
		} else {
			//stop robot once time is up
			driveRobot(0.0, 0.0);
		}
    }
    
    public void teleopInit() {
    	LiveWindow.setEnabled(false);
    	motorSpeed = 0.0;
    
    }
    
    public void teleopPeriodic() {
    	
    	/*
    	//if the A button is pressed, increase motorSpeed by 0.2
    	if(stick.getRawButton(2)) {
    		if(!aButtonPressed && (motorSpeed < 1)){
    			aButtonPressed = true;
    			motorSpeed+= 0.2;
    			
    		}
    	} else {
    		aButtonPressed= false;
    	}
    	
    	//if the Y button is pressed, decrease motorSpeed by 0.2
    	if(stick.getRawButton(4)) {
    		if(!yButtonPressed && (motorSpeed > -1)){
    			yButtonPressed = true;
    			motorSpeed-= 0.2;
    		}
    	}
    	else {
    		yButtonPressed= false;
    	}
    	*/
    	
    	//input speed from smart dashboard
    	motorSpeed = SmartDashboard.getNumber("InputSpeed");
    	
    	//drive shooter motor
    	shooter.set(motorSpeed);
    	
    	//drive robot with joystick values
    	driveRobot(-stick.getRawAxis(1), - stick.getRawAxis(3));
    	
    	//send data to Smart Dashboard
    	SmartDashboard.putNumber("Speed", motorSpeed);
    	SmartDashboard.putNumber("lDrive", lDrive1.getSpeed());
    	SmartDashboard.putNumber("rDrive", rDrive1.getSpeed());
    	SmartDashboard.putNumber("Gyro Rotation", gyro.getAngle());
    	SmartDashboard.putNumber("X Acceleration", accelerometer.getX());
    	SmartDashboard.putNumber("Y Acceleration", accelerometer.getY());
    	SmartDashboard.putNumber("Z Acceleration", accelerometer.getAcceleration(ADXL362.Axes.kZ));
    	
    	//write data to .csv file
    	writeCSV("\n" + Timer.getFPGATimestamp() + ", " + lDriveEncoder.get() + ", " + rDriveEncoder.get() + ", " + lDrive1.getSpeed() + ", " + rDrive1.getSpeed());
    	
    }
    
    public void testPeriodic() {
    	LiveWindow.run();
    }
    
    //function to write data to a .csv file
    public void writeCSV(Object data) {
    	try(PrintWriter csv = new PrintWriter(new BufferedWriter(new FileWriter("home/lvuser/Output.csv", true)))) {
    		csv.print(", " + data.toString());
    		
    	} catch(IOException e) {
    		e.printStackTrace();
    	}
    }
    
    //function to tank-drive robot given speed values
    public void driveRobot(double left, double right) {
		rDrive1.set(right);
		rDrive2.set(right);
		// Values are multiplied by -1 to ensure that the motors on the right
		// spin opposite the motors on the left.
		lDrive1.set(-left);
		lDrive2.set(-left);

	}
   	
 }