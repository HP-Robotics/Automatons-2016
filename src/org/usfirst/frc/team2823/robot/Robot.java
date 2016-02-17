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
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalInput;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

public class Robot extends IterativeRobot {
	
	//QUICKCLICK declarations
	/*declare drive-related objects and variables*/
	Encoder lDriveEncoder;
	Encoder rDriveEncoder;
	ADXRS450_Gyro gyro;

	VictorSP lDrive1;
	VictorSP lDrive2;
	VictorSP rDrive1;
	VictorSP rDrive2;
	
	ATM2016PIDController turnControl;
	
	double leftSpeed;
	double rightSpeed;
	boolean gyroDrive = false;
	boolean tankDriveEnabled = true;
	
	ToggleSwitch pidState;
	
	/*declare intake-related objects and variables*/
	Talon intake;
	
	double intakeSpeed = 0.0;
	String intakeOn = "Off";
	
	ToggleSwitch intakeState;
	ToggleSwitch intakeEnableState;
	
	/*declare shooter-related objects and variables*/
	Encoder shooterEncoder;
	
	Talon shooter;
	
	ATM2016Counter shooterCounter;
	ATM2016PIDController shooterSpeedControl;
	
	double shooterSpeed = 0.0;

	/*declare arm-related objects and variables*/
	DigitalInput upperLimitSwitch;
	DigitalInput lowerLimitSwitch;
	
	CANTalon arm;
	
	ATM2016PIDController armControl;
	
	double armSpeed = 0.0;
	
	/*declare CSV file-writing-related objects and variables*/
	File f;
	BufferedWriter bw;
	FileWriter fw;
	
	int autoCounter;
	double autoTimePrev;
	double autoPosPrev;
	double autoVelPrev;
	double initTime;
	
	/*declare joystick and button press-related objects*/
	Joystick stick;
	
	boolean aButtonPressed = false;
	boolean yButtonPressed = false;
	boolean xButtonPressed = false;
	boolean lBumperPressed = false;
	boolean lTriggerPressed = false;
	boolean rBumperPressed = false;
	boolean rTriggerPressed = false;
	
	ToggleSwitch encoderResetState;
	
	/* this class tracks a mode switch, e.g. press X to switch
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
		
		public void reset() {
			state = false;
			previousState = false;
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
	
	//QUICKCLICK robotInit
    public void robotInit() {
    	
    	//create objects based on subsystem
    	createDriveObjects();
    	createIntakeObjects();
    	createShooterObjects();
    	createArmObjects();
    	createFileWritingObjects();
    	putSmartDashboardValues();
    	
    	//create USB camera
    	//CameraServer camera;
    	//camera = CameraServer.getInstance();
    	//camera.setQuality(50);
    	//camera.startAutomaticCapture("cam0");
    	
    	//create joystick
    	stick = new Joystick(0);
    }
    
    public void autonomousInit() {
    	//initialize values for capturing data during autonomous
    	autoCounter = 0;
    	initTime = Timer.getFPGATimestamp();
    	autoTimePrev = initTime;
    	autoPosPrev = lDriveEncoder.get();
    	autoVelPrev = 0;
    	
    	//begin driving the robot
    	//driveRobot(0.2, 0.2);
    	
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
    
    //QUICKCLICK teleopInit
    public void teleopInit() {
    	LiveWindow.setEnabled(false);
    	
    	//disable shooter and intake motors
    	shooterSpeed = 0.0;
    	intakeSpeed = 0.0;
    	armSpeed = 0.0;
    	
    	arm.enable();
    	
    	// This will throw an exception if there is no Gyro and crash the robot.
    	//   We need to add a try block around this, maybe in a quick function to wrap it
    	gyroReset();
    
    }
    
    //QUICKCLICK teleopPeriodic
    public void teleopPeriodic() {
    	
    	//get PIDF values
    	//shooterSpeedControl.setPID(SmartDashboard.getNumber("P"), SmartDashboard.getNumber("I")/1000, SmartDashboard.getNumber("D"), SmartDashboard.getNumber("F"));
    	
    	//if the X button is pressed, use PID to drive shooter wheel
    	if(pidState.updateState(stick.getRawButton(1))) {
    		System.out.println("Updated button");
    		if(pidState.switchEnabled()) {
    			System.out.println("PID should be on");
    			shooterSpeedControl.enableLog("ShootPID.csv");
    			shooterSpeedControl.enable();
    			shooterSpeedControl.setSetpointInRPMs(SmartDashboard.getNumber("TargetShooterSpeed"));
    			
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
    	
    	//set the arm speed using the Y and A buttons
    	if(stick.getRawButton(4) && !upperLimitSwitch.get()){
    		armSpeed = 0.45;
    	}
    	else if(stick.getRawButton(2) && !lowerLimitSwitch.get()){
    		armSpeed = -0.4;
    	}
    	else {
    		armSpeed = 0.0;
    	}
    	
    	//set intake using left trigger and left bumper
    	if(intakeState.updateState(stick.getRawButton(5))) {
    		System.out.println("Updated button");
    		if(intakeState.switchEnabled()) {
    			intakeSpeed = -1.0;
    			intakeOn = "In";
    			
    		} else {
    			intakeSpeed = 1.0;
        		intakeOn = "Out";
    		}
    	}
    	
    	if(intakeEnableState.updateState(stick.getRawButton(7))) {
    		intakeSpeed = 0.0;
    		intakeOn = "Off";
    		intakeState.reset();
    		
    	}
    	
    	//reset arm encoder when B button is pressed
    	if(encoderResetState.updateState(stick.getRawButton(3))) {
    		arm.setEncPosition(0);
    	}
    	
    	//calculate motor speeds
    	//setIntakeSpeed();
    	setShooterSpeed();
    	
    	//input speed from smart dashboard
    	arm.set(armSpeed);    	
    	
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
    	SmartDashboard.putNumber("Left Encoder", lDriveEncoder.get());
    	SmartDashboard.putNumber("Right Encoder", rDriveEncoder.get());
    	SmartDashboard.putNumber("Shooter Encoder", shooterEncoder.get());
    	SmartDashboard.putNumber("Arm Encoder", arm.getEncPosition());
    	SmartDashboard.putBoolean("Lower Limit", lowerLimitSwitch.get());
    	SmartDashboard.putBoolean("Upper Limit", upperLimitSwitch.get());
    	SmartDashboard.putNumber("Shooter Counter", shooterCounter.get());
    	SmartDashboard.putNumber("ActualShooterSpeed", shooterCounter.getRateInRPMs());
    	SmartDashboard.putString("Intake", intakeOn);

    	
    	//update PID constants to Smart Dashboard values
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
    	}else if(stick.getPOV() == 90) {
    		if(!gyroDrive){
    			gyroReset();
    			turnControl.setSetpoint(90);
    			//turnControl.enableLog("TurnPID.csv");
    			turnControl.enable();
    			
    			gyroDrive = true;
    			tankDriveEnabled = false;
    		}
    	} else if(stick.getPOV() == 270) {
    		if(!gyroDrive){
    			gyroReset();
    			turnControl.setSetpoint(-90);
    			//turnControl.enableLog("TurnPID.csv");
    			turnControl.enable();
    			
    			gyroDrive = true;
    			tankDriveEnabled = false;
    		}
    	} else if(gyroDrive) {
    		tankDriveEnabled = true;
    		gyroDrive = false;
    		
    		turnControl.disable();
    		//turnControl.closeLog();
    		turnControl.reset();
    	}
    }
    
    //QUICKCLICK creations
    //create objects to run drive system
    public void createDriveObjects() {
    	lDriveEncoder = new Encoder(0, 1, true, EncodingType.k4X);
    	rDriveEncoder = new Encoder(2, 3, true, EncodingType.k4X);
    	lDriveEncoder.reset();
    	rDriveEncoder.reset();
    	
    	gyro = new ADXRS450_Gyro(Port.kOnboardCS0);
    	
    	//calibrate gyro
    	gyro.calibrate();
    	
    	lDrive1 = new VictorSP(0);
    	lDrive2 = new VictorSP(1);
    	rDrive1 = new VictorSP(2);
    	rDrive2 = new VictorSP(3);
    	
    	turnControl = new ATM2016PIDController(0.08, 0.0000001, 0.005, gyro, new GyroPIDOutput());
    	
    	pidState = new ToggleSwitch();
    	
    }
    
    //create objects to run intake system
    public void createIntakeObjects() {
    	intake = new Talon(4);
    	
    	intakeState = new ToggleSwitch();
    	intakeEnableState = new ToggleSwitch();
    	
    }
    
    //create objects to run shooter system
    public void createShooterObjects() {
    	shooterEncoder = new Encoder(4, 5, true, EncodingType.k4X);
    	shooterEncoder.setPIDSourceType(PIDSourceType.kRate);
    	shooterEncoder.setReverseDirection(true);
    	shooterEncoder.reset();
    	
    	shooter = new Talon(5);
    	
    	shooterCounter = new ATM2016Counter();
    	shooterCounter.setUpSource(6);
    	shooterCounter.setUpSourceEdge(true, false);
    	shooterCounter.setPIDSourceType(PIDSourceType.kDisplacement);
    	
    	shooterSpeedControl = new ATM2016PIDController(-0.0001, -0.0000005, 0.0, 0.0, shooterCounter, shooter, 0.01);
    	shooterSpeedControl.setOutputRange(-1.0, 0.0);  	
    }
    
    //create objects to run the shooter arm
    public void createArmObjects() {
    	upperLimitSwitch = new DigitalInput(9);
    	lowerLimitSwitch = new DigitalInput(8);
    	
    	arm = new CANTalon(0);
    	
    	encoderResetState = new ToggleSwitch();
    	
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
   	
    //put initial values to the SmartDashboard
    public void putSmartDashboardValues() {
    	/**
    	 * TgtShtrSpeed: 3900 up close (0 robot widths)
    	 * TgtShtrSpeed: 3500 @ 1 robot width
    	 * TgtShtrSpeed: 3400 @ 2 robot widths
    	 * TgtShtrSpeed: 3300 far away (3 robot widths)
    	 */
    	SmartDashboard.putNumber("TargetShooterSpeed", 3300);
    	SmartDashboard.putNumber("P", 0.0);
    	SmartDashboard.putNumber("I", 0.0);
    	SmartDashboard.putNumber("D", 0.0);
    	SmartDashboard.putNumber("F", 0.0);
    	SmartDashboard.putNumber("k_angle", 0.1);
    	SmartDashboard.putNumber("k_sensitivity", 0.5);
    	SmartDashboard.putNumber("arm", 0);
    	SmartDashboard.putNumber("Arm Speed", 0.0);
    	
    }
 }