package org.usfirst.frc.team2823.robot;

//import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.TalonSRX;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.communication.UsageReporting;
import edu.wpi.first.wpilibj.communication.FRCNetworkCommunicationsLibrary.tInstances;
import edu.wpi.first.wpilibj.communication.FRCNetworkCommunicationsLibrary.tResourceType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.ADXL362;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;

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
	DigitalInput limitSwitch;
	
	ATM2016PIDController leftDrivingControl;
	
	RobotDrive robot;
	
	
	File f;
	BufferedWriter bw;
	FileWriter fw;
	
	//declare class-wide variables
	int autoCounter;
	double autoTimePrev;
	double autoPosPrev;
	double autoVelPrev;
	double initTime;
	
	double shooterSpeed = 0.0;
	double leftSpeed;
	double rightSpeed;
	boolean aButtonPressed = false;
	boolean yButtonPressed = false;
	boolean xButtonPressed = false;
	boolean gyroDrive = false;
	
    public void robotInit() {
    	
    	//create USB camera
    	//CameraServer camera;
    	//camera = CameraServer.getInstance();
    	//camera.setQuality(50);
    	//camera.startAutomaticCapture("cam0");
    	
    	//create objects
    	stick = new Joystick(0);
    	lDrive1 = new VictorSP(0);
    	lDrive2 = new VictorSP(1);
    	rDrive1 = new VictorSP(2);
    	rDrive2 = new VictorSP(3);
    	shooter = new TalonSRX(4);
    	limitSwitch = new DigitalInput(9);
    	
    	lDriveEncoder = new Encoder(0, 1, true, EncodingType.k4X);
    	rDriveEncoder = new Encoder(2, 3, true, EncodingType.k4X);
    	
    	leftDrivingControl = new ATM2016PIDController(0.06, 0, 0, lDriveEncoder, new DrivePIDOutput());
    	
    	robot  = new RobotDrive(lDrive1, lDrive2, rDrive1, rDrive2);
    	
    	SmartDashboard.putNumber("InputSpeed", 0.0);
    	SmartDashboard.putNumber("P", 0.0);
    	SmartDashboard.putNumber("I", 0.0);
    	SmartDashboard.putNumber("D", 0.0);
    	SmartDashboard.putNumber("k_angle", 0.1);
    	SmartDashboard.putNumber("k_sensitivity", 0.5);
    	
    	//create gyro and accelerometer
    	gyro = new ADXRS450_Gyro(Port.kOnboardCS0);
    	accelerometer = new ADXL362(ADXL362.Range.k2G);
    	
    	//reset encoders
    	lDriveEncoder.reset();
    	rDriveEncoder.reset();
    	
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
    	//shooterSpeed = prefs.getDouble("Speed", 0.0);
    }
    
    public void autonomousInit() {
    	//initialize values for capturing data during autonomous
    	autoCounter = 0;
    	initTime = Timer.getFPGATimestamp();
    	autoTimePrev = initTime;
    	autoPosPrev = lDriveEncoder.get();
    	autoVelPrev = 0;
    	
    	//begin driving the robot
    	driveRobot(0.2, 0.2);
    	
    }

    public void autonomousPeriodic() {
    	
    	//drive the robot for a certain number of seconds
    	if((Timer.getFPGATimestamp()-initTime) < 4.0 ) {
			
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
    	
    	//disable shooter motor
    	shooterSpeed = 0.0;
    	
    	//reset gyro
    	gyro.calibrate();
    	gyro.reset();
    
    }
    
    public void teleopPeriodic() {
    	
    	/*
    	//if the A button is pressed, increase shooterSpeed by 0.2
    	if(stick.getRawButton(2)) {
    		if(!aButtonPressed && (shooterSpeed < 1)){
    			aButtonPressed = true;
    			shooterSpeed+= 0.2;
    			
    		}
    	} else {
    		aButtonPressed= false;
    	}
    	
    	//if the Y button is pressed, decrease shooterSpeed by 0.2
    	if(stick.getRawButton(4)) {
    		if(!yButtonPressed && (shooterSpeed > -1)){
    			yButtonPressed = true;
    			shooterSpeed-= 0.2;
    		}
    	}
    	else {
    		yButtonPressed= false;
    	}
    	*/
    	
    	//input speed from smart dashboard
    	shooterSpeed = SmartDashboard.getNumber("InputSpeed");
    	
    	//drive shooter motor
    	shooter.set(shooterSpeed);
    	
    	
    	
    	//if the X button is pressed, use PID to drive 500 encoder ticks
    	/*if(stick.getRawButton(1)) {
    		if(!xButtonPressed){
    			xButtonPressed = true;
    			
    			leftDrivingControl.enable();
    			leftDrivingControl.setSetpoint(lDriveEncoder.get() + 500);
    			
    		}
    	} else {
    		//otherwise disable PID and drive robot with joystick values
    		xButtonPressed = false;
    		
    		leftDrivingControl.disable();
    		driveRobot(stick.getRawAxis(1)* -0.75, stick.getRawAxis(3)* -0.75);
    		
    	}*/
    	
    	if((stick.getPOV() >= 0 && stick.getPOV() <= 45) || (stick.getPOV()>=315)) {
    		goNow(0.7, -gyro.getAngle() * SmartDashboard.getNumber("k_angle"), 0.5, 0.5);
    		if(!gyroDrive){
    			gyro.reset();
    			gyroDrive = true;
    		}
    		
    	} else if(stick.getPOV() >= 135 && stick.getPOV()<= 225) {
    		goNow(-0.7, gyro.getAngle() * SmartDashboard.getNumber("k_angle"), -0.5, 0.5);
    		if(!gyroDrive){
    			gyro.reset();
    			gyroDrive = true;
    		}
    	} else {
    		driveRobot(stick.getRawAxis(1)* -0.75, stick.getRawAxis(3)* -0.75);
    		gyroDrive = false;
    	}
    	
    	//send data to Smart Dashboard
    	SmartDashboard.putNumber("Speed", shooterSpeed);
    	SmartDashboard.putNumber("lDrive", lDrive1.getSpeed());
    	SmartDashboard.putNumber("rDrive", rDrive1.getSpeed());
    	SmartDashboard.putNumber("Gyro Rotation", gyro.getAngle());
    	SmartDashboard.putNumber("X Acceleration", accelerometer.getX());
    	SmartDashboard.putNumber("Y Acceleration", accelerometer.getY());
    	SmartDashboard.putNumber("Z Acceleration", accelerometer.getZ());
    	SmartDashboard.putNumber("Left Encoder", lDriveEncoder.get());
    	SmartDashboard.putNumber("Right Encoder", rDriveEncoder.get());
    	
    	//update PID constants to Smart Dashboard values
    	leftDrivingControl.setPID(SmartDashboard.getNumber("P"), SmartDashboard.getNumber("I")/1000, SmartDashboard.getNumber("D"));
    	
    	//write data to .csv file
    	writeCSV("\n" + Timer.getFPGATimestamp() + ", " + lDriveEncoder.get() + ", " + rDriveEncoder.get() + ", " + lDrive1.getSpeed() + ", " + rDrive1.getSpeed());
    	
    	if(limitSwitch.get()){
    		shooter.set(0.2);
    	}
    	else {
    		shooter.set(0.0);
    	}
    	SmartDashboard.putBoolean("Limit Switch", limitSwitch.get());
    	
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
		rDrive1.set(-right);
		rDrive2.set(-right);
		// Values are multiplied by -1 to ensure that the motors on the right
		// spin opposite the motors on the left.
		lDrive1.set(left);
		lDrive2.set(left);

	}
    
    public class DrivePIDOutput implements PIDOutput {

		@Override
		public void pidWrite(double output) {
			SmartDashboard.putNumber("Drive PIDOutput", output);
			lDrive1.set(-output);
			lDrive2.set(-output);
			rDrive1.set(output);
			rDrive2.set(output);
		}

	}
    public void goNow(double outputMagnitude, double curve, double minimum, double sensitivity) {
        double leftOutput, rightOutput;
        
        if (curve < 0) {
          double value = Math.log(-curve);
          double ratio = (value - sensitivity) / (value + sensitivity);
          if (ratio == 0) {
            ratio = .0000000001;
          }
          leftOutput = outputMagnitude / ratio;
          rightOutput = outputMagnitude;
        } else if (curve > 0) {
          double value = Math.log(curve);
          double ratio = (value - sensitivity) / (value + sensitivity);
          if (ratio == 0) {
            ratio = .0000000001;
          }
          leftOutput = outputMagnitude;
          rightOutput = outputMagnitude / ratio;
        } else {
          leftOutput = outputMagnitude;
          rightOutput = outputMagnitude;
        }
        if(Math.abs(leftOutput)<Math.abs(minimum)){
        	leftOutput = minimum;
        }
        if(Math.abs(rightOutput)<Math.abs(minimum)){
        	rightOutput = minimum;
        }
        driveRobot(leftOutput, rightOutput);
      }
   	
 }