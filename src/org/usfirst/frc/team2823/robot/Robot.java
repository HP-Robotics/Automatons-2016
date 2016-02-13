package org.usfirst.frc.team2823.robot;

//import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.ADXL362;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalInput;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

public class Robot extends IterativeRobot {
	//create objects
	Joystick stick;
	Encoder lDriveEncoder;
	Encoder rDriveEncoder;
	Encoder shooterEncoder;
	VictorSP lDrive1;
	VictorSP lDrive2;
	VictorSP rDrive1;
	VictorSP rDrive2;
	CANTalon arm;
	Talon shooter;
	Talon intake;
	ADXRS450_Gyro gyro;
	ADXL362 accelerometer;
	DigitalInput limitSwitch;
	
	ATM2016PIDController shooterSpeedControl;
	ATM2016PIDController turnControl;
	
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
	double intakeSpeed = 0.0;
	double leftSpeed;
	double rightSpeed;
	
	boolean aButtonPressed = false;
	boolean yButtonPressed = false;
	boolean xButtonPressed = false;
	boolean lBumperPressed = false;
	boolean lTriggerPressed = false;
	boolean rBumperPressed = false;
	boolean rTriggerPressed = false;
	boolean gyroDrive = false;
	boolean tankDriveEnabled = true;
	boolean intakeEnabled = false;
	
	ToggleSwitch pidState;
	ToggleSwitch intakeState;
	ToggleSwitch intakeEnableState;
	
	/* this class tracks a mode switch. i.e. press X to switch
	 * to PID drive, press again to switch back to tank drive */
	static class ToggleSwitch {
		private boolean state = false;
		private boolean previousState = false;
		
		/* return whether the mode switch is enabled */
		public boolean switchEnabled() {
			return state;
		}
		
		/* update the mode switch based on this tick's controller button state.
		 * returns whether the state changed */
		public boolean updateState(boolean btnState){
			boolean stateChanged = false;
			
			if(btnState && (btnState != previousState)) {
				state = !state;
				stateChanged = true;
			}
			
			previousState = btnState;
			
			return stateChanged;
		}
	}
	/*
	public static void main(String[] blah)
	{
		System.out.println("hi");
		ToggleSwitch s = new ToggleSwitch();
		System.out.println("enabled:" + s.switchEnabled());
		boolean b = s.updateState(true);
		System.out.println("switched: " + b);
		System.out.println("enabled:" + s.switchEnabled());
		b = s.updateState(false);
		System.out.println("switched: " + b);
		System.out.println("enabled:" + s.switchEnabled());
	}
	*/
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
    	shooter = new Talon(4);
    	intake = new Talon(5);
    	arm = new CANTalon(0);
    	
    	limitSwitch = new DigitalInput(9);
    	
    	lDriveEncoder = new Encoder(0, 1, true, EncodingType.k4X);
    	rDriveEncoder = new Encoder(2, 3, true, EncodingType.k4X);
    	shooterEncoder = new Encoder(4, 5, true, EncodingType.k4X);
    	
    	pidState = new ToggleSwitch();
    	intakeState = new ToggleSwitch();
    	intakeEnableState = new ToggleSwitch();
    	
    	shooterEncoder.setPIDSourceType(PIDSourceType.kRate);
    	shooterEncoder.setReverseDirection(true);
    	
    	//create gyro and accelerometer
    	gyro = new ADXRS450_Gyro(Port.kOnboardCS0);
    	accelerometer = new ADXL362(ADXL362.Range.k2G);
    	
    	//calibrate gyro
    	gyro.calibrate();
    	
    	shooterSpeedControl = new ATM2016PIDController(0.0, 0.0, 0.0, 0.0, shooterEncoder, lDrive1, 10);
    	turnControl = new ATM2016PIDController(0.08, 0.0000001, 0.005, gyro, new GyroPIDOutput());
    	
    	SmartDashboard.putNumber("InputSpeed", 0.0);
    	SmartDashboard.putNumber("P", 0.0);
    	SmartDashboard.putNumber("I", 0.0);
    	SmartDashboard.putNumber("D", 0.0);
    	SmartDashboard.putNumber("F", 0.0);
    	SmartDashboard.putNumber("k_angle", 0.1);
    	SmartDashboard.putNumber("k_sensitivity", 0.5);
    	SmartDashboard.putNumber("arm", 0);
    	
    	//reset encoders
    	lDriveEncoder.reset();
    	rDriveEncoder.reset();
    	shooterEncoder.reset();
    	
    	//create .csv file to log data
    	createCSV("AutoOutput");
    	createCSV("TeleopOutput");
    	
    	//write header to csv file
    	writeCSV("\nTimestamp, Left Encoder, Right Encoder, Left Speed, Right Speed, dT, L-Vel, L-Accel", "TeleopOutput");
    	writeCSV("\nTimestamp, Left Encoder, Right Encoder, Left Speed, Right Speed, dT, L-Vel, L-Accel", "AutoOutput");
    	
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
				
				writeCSV("\n" + currentTime + ", " + currentPos + ", " + rDriveEncoder.get() + ", " + lDrive1.getSpeed() + ", " + rDrive1.getSpeed() + ", " + dT + ", " + V + ", " + A, "AutoOutput");
				
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
    	
    	//disable shooter and intake motors
    	shooterSpeed = 0.0;
    	intakeSpeed = 0.0;
    	
    	arm.enable();
    	
    	// This will throw an exception if there is no Gyro and crash the robot.
    	//   We need to add a try block around this, maybe in a quick function to wrap it
    	gyroReset();
    
    }
    
    public void teleopPeriodic() {
    	
    	//if the X button is pressed, use PID to drive 500 encoder ticks
    	if(pidState.updateState(stick.getRawButton(1))) {
    		System.out.println("Updated button");
    		if(pidState.switchEnabled()) {
    			System.out.println("PID should be on");
    			shooterSpeedControl.enableLog("ShooterPID.csv");
    			shooterSpeedControl.enable();
    			shooterSpeedControl.setSetpoint(10000);
    			
    			tankDriveEnabled = false;
    			
    		} else {
    			System.out.println("PID should be off");
    			shooterSpeedControl.disable();
    			shooterSpeedControl.reset();
        		shooterSpeedControl.closeLog();
    			
        		tankDriveEnabled = true;
        		
    		}
    	}
    	
    	if(shooterSpeedControl.isEnabled()) {
    		SmartDashboard.putNumber("Error", shooterSpeedControl.getAvgError());
    	}
    	
    	
    	if(intakeState.updateState(stick.getRawButton(5))&&intakeEnabled) {
    		System.out.println("Updated button");
    		if(intakeState.switchEnabled()) {
    			intakeSpeed = -0.8;
    			
    		} else {
    			intakeSpeed = 0.8;
        		
    		}
    	}
    	
    	if(intakeEnableState.updateState(stick.getRawButton(7))) {
    		System.out.println("Updated button");
    		if(intakeEnableState.switchEnabled()) {
    			intakeEnabled = true;
    			
    		} else {
    			intakeEnabled = false;
    			intakeSpeed = 0.0;
    			
    		}
    	}
    	
    	//calculate motor speeds
    	//setIntakeSpeed();
    	setShooterSpeed();
    	
    	//input speed from smart dashboard
    	arm.set(SmartDashboard.getNumber("arm"));
    	
    	
    	//drive motors using calculated speeds
    	shooter.set(shooterSpeed);
    	intake.set(intakeSpeed);
    	goGyro();
    	if(tankDriveEnabled){
    		driveRobot(stick.getRawAxis(1)* -0.75, stick.getRawAxis(3)* -0.75);
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
    	SmartDashboard.putNumber("Shooter Encoder", shooterEncoder.get());
    	SmartDashboard.putNumber("Arm Encoder", arm.getEncPosition());
    	SmartDashboard.putBoolean("Limit Switch", limitSwitch.get());
    	
    	//update PID constants to Smart Dashboard values
    	//shooterSpeedControl.setPID(SmartDashboard.getNumber("P"), SmartDashboard.getNumber("I")/1000, SmartDashboard.getNumber("D"), SmartDashboard.getNumber("F"));
    	//turnControl.setPID(SmartDashboard.getNumber("P"), SmartDashboard.getNumber("I")/1000, SmartDashboard.getNumber("D"));
    	
    	//write data to .csv file
    	writeCSV("\n" + Timer.getFPGATimestamp() + ", " + lDriveEncoder.get() + ", " + rDriveEncoder.get() + ", " + lDrive1.getSpeed() + ", " + rDrive1.getSpeed(), "TeleopOutput");
    }
    
    public void testPeriodic() {
    	LiveWindow.run();
    }
    
    //function to write data to a .csv file
    public void writeCSV(Object data, String file) {
    	try(PrintWriter csv = new PrintWriter(new BufferedWriter(new FileWriter("home/lvuser/" + file + ".csv", true)))) {
    		csv.print(", " + data.toString());
    		
    	} catch(IOException e) {
    		e.printStackTrace();
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
    
    //function to tank-drive robot given speed values
    public void driveRobot(double left, double right) {
			rDrive1.set(-right);
			rDrive2.set(-right);
			// Values are multiplied by -1 to ensure that the motors on the right
			// spin opposite the motors on the left.
			lDrive1.set(left);
			lDrive2.set(left);
    	

	}
    
    public void setIntakeSpeed() {
    	//if the left trigger (bottom) is pressed, decrease intakeSpeed by 0.2
    	if(stick.getRawButton(7)) {
    		if(!lTriggerPressed && (shooterSpeed > -1)){
    			lTriggerPressed = true;
    			intakeSpeed -= 0.2;
    			
    		}
    	} else {
    		lTriggerPressed = false;
    	}
    	
    	//if the left bumper (top) is pressed, increase intakeSpeed by 0.2
    	if(stick.getRawButton(5)) {
    		if(!lBumperPressed && (shooterSpeed < 1)){
    			lBumperPressed = true;
    			intakeSpeed += 0.2;
    		}
    	}
    	else {
    		lBumperPressed = false;
    	}
    }
    
    public void setShooterSpeed() {
    	
    	//if the right trigger (bottom) is pressed, decrease shooterSpeed by 0.2
    	if(stick.getRawButton(8)) {
    		if(!rTriggerPressed && (shooterSpeed > -1)){
    			rTriggerPressed = true;
    			shooterSpeed -= 0.2;
    			
    		}
    	} else {
    		rTriggerPressed = false;
    	}
    	
    	//if the right bumper (top) is pressed, increase shooterSpeed by 0.2
    	if(stick.getRawButton(6)) {
    		if(!rBumperPressed && (shooterSpeed < 1)){
    			rBumperPressed = true;
    			shooterSpeed += 0.2;
    		}
    	}
    	else {
    		rBumperPressed = false;
    	}
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
    
    public class GyroPIDOutput implements PIDOutput {

		@Override
		public void pidWrite(double output) {
			SmartDashboard.putNumber("Gyro PIDOutput", output);
			lDrive1.set(output);
			lDrive2.set(output);
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
    
    public void gyroReset() {
    	try{
    		gyro.reset();
    	}catch(Exception e) {
    		System.out.println("Gyro not work");
    	}
    	
    }
    public void goGyro (){
    	if((stick.getPOV() >= 0 && stick.getPOV() <= 45) || (stick.getPOV()>=315)) {
    		goNow(0.7, -gyro.getAngle() * SmartDashboard.getNumber("k_angle"), 0.5, 0.5);
    		if(!gyroDrive){
    			gyroReset();
    			gyroDrive = true;
    			tankDriveEnabled = false;
    		}
    		
    	} else if(stick.getPOV() >= 135 && stick.getPOV()<= 225) {
    		goNow(-0.7, gyro.getAngle() * SmartDashboard.getNumber("k_angle"), -0.5, 0.5);
    		if(!gyroDrive){
    			gyroReset();
    			gyroDrive = true;
    			tankDriveEnabled = false;
    		}
    	} else if(stick.getPOV() == 90) {
    		if(!gyroDrive){
    			gyroReset();
    			turnControl.setSetpoint(90);
    			turnControl.enableLog("TurnPID.csv");
    			turnControl.enable();
    			
    			gyroDrive = true;
    			tankDriveEnabled = false;
    		}
    	} else if(stick.getPOV() == 270) {
    		if(!gyroDrive){
    			gyroReset();
    			turnControl.setSetpoint(-90);
    			turnControl.enableLog("TurnPID.csv");
    			turnControl.enable();
    			
    			gyroDrive = true;
    			tankDriveEnabled = false;
    		}
    	} else if(gyroDrive) {
    		tankDriveEnabled = true;
    		gyroDrive = false;
    		
    		turnControl.disable();
    		turnControl.closeLog();
    		turnControl.reset();
    	}
    }
   	
 }