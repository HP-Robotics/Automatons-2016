package org.usfirst.frc.team2823.robot;

//import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Command;

import java.io.*;
import java.net.*;

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
	Talon shooter;
	
	ATM2016Counter shooterCounter;
	ATM2016PIDController shooterSpeedControl;
	
	double shooterSpeed = 0.0;

	/*declare arm-related objects and variables*/
	DigitalInput upperLimitSwitch;
	DigitalInput lowerLimitSwitch;
	
	CANTalon arm;
	
	CANPIDSource armEncoder;
	ATM2016PIDController armControl;
	
	ToggleSwitch armState;
	
	double armSpeed = 0.0;
	boolean manualArmEnabled = true;
	
	/*declare trigger-related objects and variables*/
	Servo trigger;
	
	/*declare auto-related objects*/
	Command autoCommand;
	SendableChooser autoChooser;
	

	/*declare joystick and button press-related objects*/
	Joystick stick;
	
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
	
	/*public static void main(String[] blah)
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
		
		getHttpData("https://httpbin.org/get");
	}*/
	
	
	//QUICKCLICK robotInit
    public void robotInit() {
    	
    	//create objects based on subsystem
    	createDriveObjects();
    	createIntakeObjects();
    	createShooterObjects();
    	createArmObjects();
    	putSmartDashboardValues();
    	createAutoModes();
    	createTriggerObjects();
    	
    	//create USB camera
    	//CameraServer camera;
    	//camera = CameraServer.getInstance();
    	//camera.setQuality(50);
    	//camera.startAutomaticCapture("cam0");
    	
    	//create joystick
    	stick = new Joystick(0);
    	
    }
    
    public void autonomousInit() {
    	((AutoMode) autoChooser.getSelected()).autoInit();
    	
    }
    
    public void autonomousPeriodic() {
		((AutoMode) autoChooser.getSelected()).autoPeriodic();

    }
    
    //QUICKCLICK teleopInit
    public void teleopInit() {
    	LiveWindow.setEnabled(false);
    	
    	//disable shooter, intake and arm motors
    	shooterSpeed = 0.0;
    	intakeSpeed = 0.0;
    	armSpeed = 0.0;
    	
    	arm.enableBrakeMode(false);
    	arm.enable();
    	
    	//reset gyro
    	gyroReset();
    
    }
    
    //QUICKCLICK teleopPeriodic
    public void teleopPeriodic() {
    	
    	//run the trigger only if the shooter wheel is at speed
    	if(stick.getRawButton(3) && shooterIsAtSpeed()){
    		trigger.setAngle(80);
    	}
    	else{
    		trigger.setAngle(110);
    	}
    	
    	/*//reset arm encoder when B button is pressed
    	if(encoderResetState.updateState(stick.getRawButton(3))) {
    		armEncoder.reset();
    	}*/
    	
    	//use A button to enable/disable arm PID
    	if(armState.updateState(stick.getRawButton(2))) {
    		if(armState.switchEnabled()) {
    			//turn PID on and drive to setpoint
    			System.out.println("Arm PID should be on");
    			armControl.enableLog("armPID.csv");
    			armControl.enable();
    			armControl.setSetpoint(SmartDashboard.getNumber("Arm Target"));
    			
    			manualArmEnabled = false;
    			
    		} else {
    			System.out.println("Arm PID should be off");
    			armControl.disable();
    			armControl.reset();
    			armControl.closeLog();
    			
    			manualArmEnabled = true;
    		}
    	}
    	
    	//calculate motor speeds
    	setIntakeSpeed();
    	setShooterSpeed();
    	
    	//if manual drive of the arm is allowed, set the arm speed
    	if(manualArmEnabled) {
    		setArmSpeed();
    	}
    	//drive motors using calculated speeds
    	intake.set(intakeSpeed);
    	shooter.set(shooterSpeed);
    	arm.set(armSpeed);  
    	
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
    	SmartDashboard.putNumber("Arm Encoder", arm.getEncPosition());
    	SmartDashboard.putNumber("Shooter Counter", shooterCounter.get());
    	SmartDashboard.putNumber("ActualShooterSpeed", shooterCounter.getRateInRPMs());
    	SmartDashboard.putBoolean("Lower Limit", lowerLimitSwitch.get());
    	SmartDashboard.putBoolean("Upper Limit", upperLimitSwitch.get());
    	SmartDashboard.putString("Intake", intakeOn);
    	
    	if(shooterSpeedControl.isEnabled()) {
    		SmartDashboard.putNumber("Error", shooterSpeedControl.getAvgError());
    	}
    	
    	//update PID constants to Smart Dashboard values
    	//turnControl.setPID(SmartDashboard.getNumber("P"), SmartDashboard.getNumber("I")/1000, SmartDashboard.getNumber("D"));
    	armControl.setPID(SmartDashboard.getNumber("P"), SmartDashboard.getNumber("I")/1000, SmartDashboard.getNumber("D"), SmartDashboard.getNumber("F"));
    	
    }
    
    public void testPeriodic() {
    	LiveWindow.run();
    }
    
   
  
    
    //function to tank-drive robot given speed values
    public void driveRobot(double left, double right) {
    		// Values are multiplied by -1 to ensure that the motors on the right spin opposite the motors on the left.
			rDrive1.set(-right);
			rDrive2.set(-right);
			
			lDrive1.set(left);
			lDrive2.set(left);
    	

	}
    
    public boolean shooterIsAtSpeed() {
    	double actualSpeed = Math.abs(SmartDashboard.getNumber("ActualShooterSpeed"));
    	double targetSpeed = Math.abs(SmartDashboard.getNumber("TargetShooterSpeed"));
    	
    	if(Math.abs(targetSpeed - actualSpeed) < 100) {
    		return true;
    	}
    	
    	return false;
    }
    
    //QUICKCLICK set motor speeds
    public void setIntakeSpeed() {
    	//set intake using left trigger and left bumper
    	if(intakeState.updateState(stick.getRawButton(7))) {
    		System.out.println("Updated button");
    		if(intakeState.switchEnabled()) {
    			intakeSpeed = -1.0;
    			intakeOn = "In";
    			
    		} else {
    			intakeSpeed = 1.0;
        		intakeOn = "Out";
    		}
    	}
    	
    	if(intakeEnableState.updateState(stick.getRawButton(8))) {
    		intakeSpeed = 0.0;
    		intakeOn = "Off";
    		intakeState.reset();
    		
    	}
    }
    
    public void setShooterSpeed() {
    	//if the X button is pressed, use PID to drive shooter wheel
    	if(pidState.updateState(stick.getRawButton(1))) {
    		System.out.println("Updated button");
    		if(pidState.switchEnabled()) {
    			System.out.println("Shooter PID should be on");
    			shooterSpeedControl.enableLog("ShootPID.csv");
    			shooterSpeedControl.enable();
    			shooterSpeedControl.setSetpointInRPMs(SmartDashboard.getNumber("TargetShooterSpeed"));
    			
    			tankDriveEnabled = false;
    			
    		} else {
    			System.out.println("Shooter PID should be off");
    			shooterSpeedControl.disable();
    			shooterSpeedControl.reset();
        		shooterSpeedControl.closeLog();
    			
        		tankDriveEnabled = true;
        		
    		}
    	}
    }
    
    public void setArmSpeed() {
    	//set the arm speed using the Y and A buttons
    	if(stick.getRawButton(6)/* && !upperLimitSwitch.get()*/){
    		//armSpeed = 0.55;
    		armSpeed = SmartDashboard.getNumber("Arm Speed");
    	}
    	else if(stick.getRawButton(5)/* && !lowerLimitSwitch.get()*/){
    		//armSpeed = -0.45;
    		armSpeed = -SmartDashboard.getNumber("Arm Speed");
    	}
    	else {
    		armSpeed = 0.0;
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
    
    //QUICKCLICK getHttpData
    public static void getHttpData(String address) {
    	try {
    		URL url = new URL(address);
    		
    		try {
    			HttpURLConnection connection = (HttpURLConnection) url.openConnection();
    			connection.setRequestMethod("GET");
    			connection.connect();
    			
    			InputStream stream = connection.getInputStream();
    			
    			 BufferedReader in = new BufferedReader(new InputStreamReader(stream));
    			 
    			 String inputLine;
    			 while((inputLine = in.readLine()) != null) {
    				 System.out.println(inputLine);
    			 }
    			 
    			 in.close();
    			
    		} catch(IOException e) {}
    		
    	} catch(MalformedURLException e) {}
    }
    
    //QUICKCLICK creations
    //create objects to run drive system
    public void createDriveObjects() {
    	lDriveEncoder = new Encoder(2, 3, true, EncodingType.k4X);
    	rDriveEncoder = new Encoder(0, 1, true, EncodingType.k4X);
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
    	
    }
    
    //create objects to run intake system
    public void createIntakeObjects() {
    	intake = new Talon(4);
    	
    	intakeState = new ToggleSwitch();
    	intakeEnableState = new ToggleSwitch();
    	
    }
    
    //create objects to run shooter system
    public void createShooterObjects() {
    	
    	shooter = new Talon(5);
    	
    	shooterCounter = new ATM2016Counter();
    	shooterCounter.setUpSource(4);
    	shooterCounter.setUpSourceEdge(true, false);
    	shooterCounter.setPIDSourceType(PIDSourceType.kDisplacement);
    	
    	shooterSpeedControl = new ATM2016PIDController(-0.0001, -0.0000005, 0.0, 0.0, shooterCounter, shooter, 0.01);
    	shooterSpeedControl.setOutputRange(-1.0, 0.0);  
    	
    	pidState = new ToggleSwitch();
    }
    
    //create objects to run the arm system
    public void createArmObjects() {
    	upperLimitSwitch = new DigitalInput(8);
    	lowerLimitSwitch = new DigitalInput(7);
    	
    	arm = new CANTalon(0);
    	
    	armEncoder = new CANPIDSource(arm);
    	armControl = new ATM2016PIDController(0.0, 0.0, 0.0, 0.0, armEncoder, arm, 0.01);
    	
    	armState = new ToggleSwitch();
    	encoderResetState = new ToggleSwitch();
    	
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
    	
    	SmartDashboard.putNumber("Arm Target", 0.0);
    	
    	
    }
    
    //create autoChooser and add auto modes
    /**WARNING! If any entries are added to/removed from the SendableChooser, the robot AND the driver station computer must be simultaneously rebooted!
     * This comes from the NetworkTables being stored on both the driver station and the roboRIO
     * **/
    
    public void createAutoModes() {
    	
    	autoChooser = new SendableChooser();
    	
		autoChooser.addObject("Empty: Do Nothing", new EmptyAuto(this));
		autoChooser.addObject("Drive Back", new MoveBackAuto(this));
		autoChooser.addDefault("Calibrate", new CalibrateAuto(this));
		
    	SmartDashboard.putData("Autonomous Mode", autoChooser);
		
    }
    
    //create objects to run the trigger system
    public void createTriggerObjects() {
    	trigger = new Servo(6);
    	trigger.setAngle(110);
    }
 }