package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalInput;

import java.io.*;
import java.net.*;
import java.sql.Time;

public class Robot extends IterativeRobot {
	
	//QUICKCLICK declarations
	
	/*declare constants*/
	static final int CALIBRATIONOFFSET = 40;
	
	static final int SHOOTSETPOINT = (0 + CALIBRATIONOFFSET);
	static final int HIGHTRAVELSETPOINT = (600 + CALIBRATIONOFFSET);
	static final int MIDSETPOINT = (1800 + CALIBRATIONOFFSET);
	static final int LOWTRAVELSETPOINT = (2280 + CALIBRATIONOFFSET);
	static final int INTAKESETPOINT = (2400 + CALIBRATIONOFFSET);
	static final int OFFSET = 100;
	
	static final double FARSPEED = 3300.0;
	static final double MIDSPEED = 3200.0;
	static final double CLOSESPEED = 3425.0;
	
	static final double LEFTGOALDISTANCE = 13.25;
	static final double CAMERAANGLE = 38;
	static final double THRESHOLD_VISION_ANGLE = 10;
	static final double VISION_WAIT_TIME = 0.5;
	
	static final int TRIGGEROFFPOSITION = 95;
	static final int TRIGGERONPOSITION = 65;
	
	static final int XBUTTON = 1;
	static final int ABUTTON = 2;
	static final int BBUTTON = 3;
	static final int YBUTTON = 4;
	static final int LBUMPER = 5;
	static final int RBUMPER = 6;
	static final int LTRIGGER = 7;
	static final int RTRIGGER = 8;
	static final int STARTBUTTON = 10;
	
	static final int LEFTAXIS = 1;
	static final int RIGHTAXIS = 3;
	
	static final double DRIVETHRESHOLD = 0.05;
	
	static final double MAXACCELERATION = 275;	//inches / sec^2
	static final double MAXVELOCITY = 120;		//inches / sec

	final static double[] LEFTVOLTAGES = new double[] { 2.0, 1.79, 1.64, 1.38, 1.24, 1.13, 1.03, 0.93, 0.88, 0.82, 0.78, 0.73, 0.67, 0.61, 0.58,
														0.56, 0.54, 0.52, 0.50, 0.49, 0.48 };
	final static double[] LEFTDISTANCES = new double[] { -0.5, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 14, 16, 18, 20, 24, 28, 32, 36 };

	final static double[] RIGHTVOLTAGES = new double[] { 2.0, 1.80, 1.50, 1.37, 1.22, 1.11, 1.02, 0.90, 0.89, 0.83, 0.78, 0.73, 0.69, 0.63, 0.60,
														0.56, 0.53, 0.460, 0.39, 0.37, 0.34 };
	final static double[] RIGHTDISTANCES = new double[] { -0.5, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 14, 16, 18, 20, 24, 28, 32, 36 };
	
	/*declare drive-related objects and variables*/
	Encoder lDriveEncoder;
	Encoder rDriveEncoder;
	ADXRS450_Gyro gyro;
	AnalogInput leftIR;
	AnalogInput rightIR;
	
	VictorSP lDrive1;
	VictorSP lDrive2;
	VictorSP rDrive1;
	VictorSP rDrive2;
	
	ATM2016PIDController turnControl;
	ATM2016PIDController visionTurnControl;
	ATM2016PIDController gyroDriveControl;
	ATM2016PIDController motionDriveControl;
	
	double leftSpeed;
	double rightSpeed;
	boolean gyroDrive = false;
	boolean drivePIDEnabled = false;
	boolean motionDriveEnabled = false;
	boolean tankDriveEnabled = true;
	boolean slowDriveEnabled = false;
	boolean emergencyMode = false;
	
	static final double ENCODER_RESOLUTION = 2048;
	static final double FUDGE_FACTOR = (194.0/196.0);
	static final double R = (5.875 * FUDGE_FACTOR);
	//static final double DRIVE_RATIO = (1.786 / 4.032);
	static final double DRIVE_RATIO = (1.432 / 3.826);
	
	ToggleSwitch pidState;
	ToggleSwitch emergencyState;
	
	/*declare intake-related objects and variables*/
	Talon intake;
	
	double intakeSpeed = 0.0;
	String intakeOn = "Off";
	
	ToggleSwitch intakeInState;
	ToggleSwitch intakeOutState;
	ToggleSwitch intakeOffState;
	
	/*declare shooter-related objects and variables*/	
	Talon shooter;
	
	ATM2016Counter shooterCounter;
	ATM2016PIDController shooterSpeedControl;
	
	ToggleSwitch RPMState;
	ToggleSwitch visionShootState;
	
	boolean shootingWithVision = false;
	boolean calculatedShotDistance = false;
	boolean clearedVisionAverage = false;
	boolean atShotPosition = false;
	boolean visionShotInProgress = false;
	double shooterSpeed = 0.0;
	double visionShotSpeed = 0.0;
	int currentTargetRPM = 1;
	double[] shooterTargetRPMs = {FARSPEED, MIDSPEED, CLOSESPEED};
	String[] shooterTargetNames = {"Far Away", "Mid", "Close Up"};
	
	/*declare arm-related objects and variables*/
	DigitalInput upperLimitSwitch;
	DigitalInput lowerLimitSwitch;
	
	CANTalon arm;
	
	Encoder armEncoder;
	ATM2016PIDController armControl;
	
	ToggleSwitch armUpState;
	ToggleSwitch armDownState;
	
	double armSpeed = 0.0;
	int currentSetpoint = 0;
	int[] setpoints = {SHOOTSETPOINT, HIGHTRAVELSETPOINT, MIDSETPOINT, LOWTRAVELSETPOINT, INTAKESETPOINT};
	String[] setpointNames = {"Shoot", "High Travel", "Mid", "Low Travel", "Intake"};
	
	/*declare trigger-related objects and variables*/
	Servo trigger;
	
	boolean firingInProgress = false;
	
	/*declare portcullis-thingy related objects and variables*/
	CANTalon portcullisArm;
	ToggleSwitch portcullisState;
	
	double buttonInitTime = 0;

	
	/*declare auto-related objects*/
	SendableChooser autoChooser;
	
	/*declare test mode-related objects*/
	TestMode testMode;
	
	boolean turn = false;
	
	/*declare Raspberry Pi communications-related objects*/
	PiThread pi;
	
	double cameraToGoalDistance = 0.0;
	double cameraToGoalAngle = 0.0;
	double cameraToLeftEdge = 0.0;
	double cameraToRightEdge = 0.0;
	double lastPiMessage = 0.0;
	double stopTime = 0.0;
	
	boolean piIsStarted = false;
	boolean setStopTime = false;
	
	/*declare joystick and button press-related objects*/
	Joystick stick1;
	Joystick stick2;
	
	
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
    	
    	//create test mode
    	testMode = new TestMode(this);
    	
    	//create USB camera
    	CameraServer camera;
    	camera = CameraServer.getInstance();
    	camera.setQuality(50);
    	camera.startAutomaticCapture("cam0");
    	
    	//create joystick
    	stick1 = new Joystick(0);
    	stick2 = new Joystick(1);
    	
    	portcullisArm = new CANTalon(1);
    	portcullisState = new ToggleSwitch();
    	
    }
    
    public void autonomousInit() {
    	//try to connect to Pi
    	if(!piIsStarted) {
    		pi = new PiThread();
    		pi.start();
    		
    		piIsStarted = true;
    	}
    	
    	//reset arm encoder and set PID target to 0 (keeps the arm upright)
    	armEncoder.reset();
    	
    	armControl.setSetpoint(0);
    	armControl.enable();
    	
    	((AutoMode) autoChooser.getSelected()).autoInit();
    	
    }
    
    public void autonomousPeriodic() {
		((AutoMode) autoChooser.getSelected()).autoPeriodic();
		
    }
    
    //QUICKCLICK teleopInit
    public void teleopInit() {
    	LiveWindow.setEnabled(false);
    	
    	//try to connect to Pi
    	if(!piIsStarted) {
    		pi = new PiThread();
    		pi.start();
    		
    		piIsStarted = true;
    	}
    	
    	//disable shooter, intake and arm motors
    	shooterSpeed = 0.0;
    	intakeSpeed = 0.0;
    	armSpeed = 0.0;
    	
    	//reset encoders
    	lDriveEncoder.reset();
    	rDriveEncoder.reset();
    	
    	//set arm PID to hold
    	// TODO JUST EXPERIMENTING
    	//armControl.enable();
    	
    	//disable arm braking
    	arm.enableBrakeMode(false);
    	arm.enable();
    	
    	//lock arm to current setpoint
    	armControl.enable();
    	
    	
    	//reset gyro
    	gyroReset();
    	
    	//disable autonomous PID
    	gyroDriveControl.disable();
    	gyroDriveControl.setOutputRange(-1.0, 1.0);
    
    }
    
    //QUICKCLICK teleopPeriodic
    public void teleopPeriodic() {
    	
    	runTrigger();
    	shootWithMagic();
    	
    	//set tank drive or slow drive based on the height of the arm
    	if (!gyroDrive && !motionDriveEnabled) {
    		if(armEncoder.get() < (MIDSETPOINT - OFFSET)) {
    			tankDriveEnabled = false;
    			slowDriveEnabled = true;
    			
    		} else if(armEncoder.get() > (MIDSETPOINT - OFFSET)) {
    			tankDriveEnabled = true;
    			slowDriveEnabled = false;
    		}
    	}
    	
    	if(emergencyState.updateState(stick1.getRawButton(STARTBUTTON) || stick2.getRawButton(STARTBUTTON))) {
    		if(emergencyState.switchEnabled()) {
    			emergencyMode = true;
    			disableArmPid();
    		} else {
    			emergencyMode = false;
    		}
    	}
    	
    	//calculate motor speeds
    	setIntakeSpeed();
    	setShooterSpeed();
    	setPtcArmSpeed();
    	
    	//if manual drive of the arm is allowed, set the arm speed
    	if(emergencyMode) {
    		setArmSpeed();
        	arm.set(armSpeed);  
    	} else {
    		setArmSetpoint();
    	}
    	
    	//calculate drive speeds
    	leftSpeed = (Math.abs(stick1.getRawAxis(LEFTAXIS)) < DRIVETHRESHOLD ? 0.0 : stick1.getRawAxis(LEFTAXIS));
    	rightSpeed = (Math.abs(stick1.getRawAxis(RIGHTAXIS)) < DRIVETHRESHOLD ? 0.0 : stick1.getRawAxis(RIGHTAXIS));
    	
    	leftSpeed = Math.pow(-leftSpeed, 3.0);
    	rightSpeed = Math.pow(-rightSpeed, 3.0);
    	
    	//drive motors using calculated speeds
    	intake.set(intakeSpeed);
    	shooter.set(shooterSpeed);
    	
    	goGyro();
    	//QUICKCLICK tank drive
    	
    	if(tankDriveEnabled ) {
    		driveRobot(leftSpeed, rightSpeed);
    	} else if(slowDriveEnabled) {
    		if(emergencyMode) {
    			driveRobot(leftSpeed, rightSpeed);
    		} else {
    			driveRobot(leftSpeed * 0.2, rightSpeed * 0.2);
    		}
    	}
    	
    	//send data to Smart Dashboard
    	/**
    	 * TgtShtrSpeed: 3900 up close (0 robot widths)
    	 * TgtShtrSpeed: 3200 @ 1 robot width
    	 * TgtShtrSpeed: 3300 @ 2 robot widths
    	 * TgtShtrSpeed: 3300 far away (3 robot widths)
    	 */
    	SmartDashboard.putString("ShooterTargetRPM", shooterTargetRPMs[currentTargetRPM] + "( " + shooterTargetNames[currentTargetRPM] + " )");
    	SmartDashboard.putString("Arm Setpoint", setpointNames[currentSetpoint] + "( " + setpoints[currentSetpoint]+ " )");
    	SmartDashboard.putNumber("Speed", shooterSpeed);
    	SmartDashboard.putNumber("lDrive", lDrive1.getSpeed());
    	SmartDashboard.putNumber("rDrive", rDrive1.getSpeed());
    	SmartDashboard.putNumber("Gyro Rotation", gyro.getAngle());
    	SmartDashboard.putNumber("Left Encoder", lDriveEncoder.get());
    	SmartDashboard.putNumber("Right Encoder", rDriveEncoder.get());
    	SmartDashboard.putNumber("Left Encoder (Inches)", driveEncoderToInches(lDriveEncoder.get()));
    	SmartDashboard.putNumber("Arm Encoder", armEncoder.get());
    	SmartDashboard.putNumber("Shooter Counter", shooterCounter.get());
    	SmartDashboard.putNumber("ActualShooterSpeed", shooterCounter.getRateInRPMs());
    	SmartDashboard.putNumber("Left IR (Raw)", leftIR.getVoltage());
    	SmartDashboard.putNumber("Left IR (Inches)", voltageToDistance(leftIR.getVoltage(), LEFTVOLTAGES, LEFTDISTANCES));
    	SmartDashboard.putNumber("Right IR (Raw)", rightIR.getVoltage());
    	SmartDashboard.putNumber("Right IR (Inches)", voltageToDistance(rightIR.getVoltage(), RIGHTVOLTAGES, RIGHTDISTANCES));
    	SmartDashboard.putBoolean("Lower Limit", lowerLimitSwitch.get());
    	SmartDashboard.putBoolean("Upper Limit", upperLimitSwitch.get());
    	SmartDashboard.putString("Intake", intakeOn);
    	SmartDashboard.putString("Arm Setpoint", setpointNames[currentSetpoint]);
    	
    	//print Pi data to SmartDashboard
    	String[] a;
    	if(pi.getLast() != null) {
    		a = pi.getLast().split(" ");
    	} else {
    		a = new String[4];
    		a[0] = "BAD";
    		a[2] = "0";
    		a[3] = "0";
    	}
    	
    	SmartDashboard.putString("Is vision good", a[0]);
    	SmartDashboard.putString("Camera-to-goal distance", a[3]);
    	SmartDashboard.putString("Camera-to-left-side distance", a[2]);
    	
    	if(shooterSpeedControl.isEnabled()) {
    		SmartDashboard.putNumber("Error", shooterSpeedControl.getAvgError());
    	}
    	
    	//update PID constants to Smart Dashboard values
    	//turnControl.setPID(SmartDashboard.getNumber("P"), SmartDashboard.getNumber("I"), SmartDashboard.getNumber("D"));
    	//armControl.setPID(SmartDashboard.getNumber("P"), SmartDashboard.getNumber("I"), SmartDashboard.getNumber("D"));
    	//gyroDriveControl.setPID(SmartDashboard.getNumber("P"), SmartDashboard.getNumber("I"), SmartDashboard.getNumber("D"), SmartDashboard.getNumber("F"));
    	motionDriveControl.setPID(SmartDashboard.getNumber("P"), SmartDashboard.getNumber("I"), SmartDashboard.getNumber("D"));
    	
    	if((Timer.getFPGATimestamp()-lastPiMessage) > 1){
    		TalkToPi.rawCommand("HELLO");
    		lastPiMessage = Timer.getFPGATimestamp();
    	}
    }
    
    //QUICKCLICK testMode
    public void testPeriodic() {
    	LiveWindow.run();
    	testMode.testPeriodic();
    }
    
    public void testInit() {
    	testMode.testInit();
    }
    
    //function to tank-drive robot given speed values
    public void driveRobot(double left, double right) {
    		// Values are multiplied by -1 to ensure that the motors on the right spin opposite the motors on the left.
			rDrive1.set(-right);
			rDrive2.set(-right);
			
			lDrive1.set(left);
			lDrive2.set(left);
    	
			
	}
    
    public boolean shooterIsAtSpeed(double threshold) {
    	double actualSpeed = Math.abs(shooterCounter.getRateInRPMs());
    	double targetSpeed = Math.abs(shooterSpeedControl.getSetpointInRPMs());
    	
    	if(Math.abs(targetSpeed - actualSpeed) < threshold) {
    		return true;
    	}
    	
    	return false;
    }
    
    //QUICKCLICK set motor speeds
    public void setIntakeSpeed() {
    	//set intake using dpad
    	 if(intakeInState.updateState((stick1.getPOV() >= 135 && stick1.getPOV()<= 225) || (stick2.getPOV() >= 135 && stick2.getPOV()<= 225))) {
    			intakeSpeed = -1.0;
    			intakeOn = "In";
    			
    	}
    	
    	if(intakeOutState.updateState((stick1.getPOV() >= 0 && stick1.getPOV() <= 45 || stick1.getPOV()>=315) || (stick2.getPOV() >= 0 && stick2.getPOV() <= 45 || stick2.getPOV()>=315))) {

    		intakeSpeed = 1.0;
    		intakeOn = "Out";
    	}
    	
    	if(intakeOffState.updateState((stick1.getPOV() == 90 || stick1.getPOV() == 270) || (stick2.getPOV() == 90 || stick2.getPOV() == 270))) {
    		turnIntakeOff();
    	}
    }
    
    public void turnIntakeOff() {
		intakeSpeed = 0.0;
		intakeOn = "Off";
    }
    
    public void setPtcArmSpeed() {
    		
    	if(portcullisState.updateState(stick2.getRawButton(LTRIGGER))){
    		buttonInitTime = Timer.getFPGATimestamp();
    	}
    	
    	if (stick2.getRawButton(LBUMPER)){
    		portcullisArm.set(0.3);
    	}
    	else if (stick2.getRawButton(LTRIGGER)){
    		double currentTime = Timer.getFPGATimestamp();
    		if ((currentTime - buttonInitTime) < 2) {
    			portcullisArm.set(-0.3);
    		}else {
    			portcullisArm.set(-0.1);
    		}
    	}else {
    		portcullisArm.set(0.0);
    	}
    }
    
    public void setShooterSpeed() {
    	//if the X button is pressed, use PID to drive shooter wheel
    	if(pidState.updateState(stick1.getRawButton(XBUTTON) || stick2.getRawButton(XBUTTON))) {
    		System.out.println("Updated button");
    		if(pidState.switchEnabled()) {
    			System.out.println("Shooter PID should be on");
    			shooterSpeedControl.enableLog("ShootPID.csv");
    			shooterSpeedControl.enable();
    			shooterSpeedControl.setSetpointInRPMs(shooterTargetRPMs[currentTargetRPM]);
    			//shooterSpeedControl.setSetpointInRPMs(SmartDashboard.getNumber("TargetShooterSpeed"));
    			
    			//tankDriveEnabled = false;
    			
    		} else {
    			System.out.println("Shooter PID should be off");
    			shooterSpeedControl.disable();
    			shooterSpeedControl.reset();
        		shooterSpeedControl.closeLog();
    			
        		//tankDriveEnabled = true;
        		
    		}
    	}
    	if(RPMState.updateState(stick1.getRawButton(YBUTTON)||stick2.getRawButton(YBUTTON))){
    		currentTargetRPM = (currentTargetRPM + 1) % shooterTargetRPMs.length;
    		shooterSpeedControl.setSetpointInRPMs(shooterTargetRPMs[currentTargetRPM]);
    		TalkToPi.rawCommand("RPM " + shooterTargetRPMs[currentTargetRPM]);
    		lastPiMessage = Timer.getFPGATimestamp();
    	}
    }
    //QUICKCLICK shoot with magic
    public void shootWithMagic () {
    	//run the *magic* button code to read data from the Pi, drive to correct distance, spin up shooter to correct RPM, then fire
    	if((stick1.getRawButton(ABUTTON) || stick2.getRawButton(ABUTTON))) {
    		if(!shootingWithVision) {
				System.err.println("A BUTTON PRESSED");
    			String piData = pi.getLast();
    	    	if(piData == null){
    	    		return;
    	    	}
    			
    			shootingWithVision = true;
    			
    			//get data from Pi
    			if(piData.contains("GOOD")) {
    				
    				motionDriveEnabled = true;
        			tankDriveEnabled= false;
        			slowDriveEnabled = false;

    				String[] piDataArray = piData.split(" ");
    			
    				cameraToGoalAngle = Double.parseDouble(piDataArray[1]);
    				
    				//check if robot is within an acceptable angle range
    				setStopTime = true;
    				
    				if(Math.abs(cameraToGoalAngle) > THRESHOLD_VISION_ANGLE) {
        				gyroReset();
    					visionTurnControl.setSetpoint(cameraToGoalAngle);
    					System.err.println("CAMERA ANGLE " + cameraToGoalAngle);
    					visionTurnControl.enable();
    					
    					stopTime = Timer.getFPGATimestamp() + 1.3;

    				}else {
    					stopTime = Timer.getFPGATimestamp() + 0.6;

    				}
    			}
    		}
    		
    		//a moment before the stopTime has finished, clear the camera running average
    		if(Timer.getFPGATimestamp() > (stopTime - 0.6) && setStopTime && !clearedVisionAverage) {
    			clearedVisionAverage = true;
    			
    			visionTurnControl.disable();
    			
    			//clear Pi running average
				TalkToPi.rawCommand("CLEAR");
    			lastPiMessage = Timer.getFPGATimestamp();
    			
    		}
    		
    		if(Timer.getFPGATimestamp()  > stopTime && setStopTime && !calculatedShotDistance) {
				calculatedShotDistance = true;

				//re-get data from Pi after waiting
				String piData = pi.getLast();
				
				//stop magic shooting if the goal is bad
				if(piData == null || piData.contains("BAD")) {
    	    		return;
    	    	}
				
				String[] piDataArray = piData.split(" ");
    			
				cameraToGoalAngle = Double.parseDouble(piDataArray[1]);
				cameraToLeftEdge = Double.parseDouble(piDataArray[2]);
				cameraToGoalDistance = Double.parseDouble(piDataArray[3]);
				cameraToRightEdge = Double.parseDouble(piDataArray[4]);
				
				if(Math.abs(cameraToGoalAngle) > THRESHOLD_VISION_ANGLE) {
					System.err.println("The vision data has not updated, quitting");
					return;
				}
				
    			//calculate distance to wall
    			double distanceToWall = cameraToGoalDistance * Math.cos(CAMERAANGLE);
    			
    			//calculate shooter RPM based on distance to wall
    			//placeholder value
    			visionShotSpeed = 3300;
    			
    			//spin up shooter wheel
    			shooterSpeedControl.setSetpoint(visionShotSpeed);
    			
    			//calculate target for motion profile
				double targetPosition = -(LEFTGOALDISTANCE - cameraToLeftEdge);
    			lDriveEncoder.reset();
    			rDriveEncoder.reset();
    			
				//TODO if the target is less than a threshold value, don't try to move
				//System.out.println(targetPosition + "\t" + (Robot.MAXVELOCITY/3) + "\t" + (Robot.MAXACCELERATION/5));
				motionDriveControl.configureGoal(targetPosition, Robot.MAXVELOCITY/3, Robot.MAXACCELERATION/5);
				System.err.println("TARGET " + (targetPosition));
				motionDriveControl.enable();
			}
    		
    		if(motionDriveControl.isPlanFinished() && !atShotPosition) {
    			atShotPosition = true;
    			
    			//raise the arm
    			disableArmPid();
    			armControl.setSetpoint(SHOOTSETPOINT);
    			enableArmPid();
    		}
    		
    		if((armEncoder.get() < (SHOOTSETPOINT + OFFSET)) && /*shooterIsAtSpeed(visionShotSpeed) && */!visionShotInProgress) {
    			visionShotInProgress = true;
    			
    			//try to shoot if the arm is above the high travel setpoint and the shooter is at speed
    			trigger.setAngle(TRIGGERONPOSITION);
    			
    			TalkToPi.rawCommand("WATCH");
    			lastPiMessage = Timer.getFPGATimestamp();
    		}
    		
    	} else {
    		//stop the *magic* shooting if the button is released
    		shootingWithVision = false;
    		calculatedShotDistance = false;
    		clearedVisionAverage = false;
    		atShotPosition = false;
    		visionShotInProgress = false;
    		motionDriveEnabled = false;
    		setStopTime = false;
    		
    		//reset the trigger
    		trigger.setAngle(TRIGGEROFFPOSITION);
    		
    		//lower the arm to the previous setpoint
    		disableArmPid();
    		armControl.setSetpoint(setpoints[currentSetpoint]);
    		enableArmPid();
    		
    		motionDriveControl.disable();
			visionTurnControl.disable();

    	}
    	
    }
    
    public void setArmSpeed() {
    	//set the arm speed using the left and right bumper
    	if(stick1.getRawButton(RBUMPER)/* && !upperLimitSwitch.get()*/){
    		armSpeed = -SmartDashboard.getNumber("Arm Speed");
    	}
    	else if(stick1.getRawButton(RTRIGGER)/* && !lowerLimitSwitch.get()*/){
    		armSpeed = SmartDashboard.getNumber("Arm Speed");
    	}
    	else {
    		armSpeed = 0.0;
    	}
    }
    
    public void setArmSetpoint() {
    	//raise arm to next setpoint, unless arm is at 90 (shoot setpoint)
    	if(armUpState.updateState(stick1.getRawButton(RBUMPER)|| stick2.getRawButton(RBUMPER))) {
    		if(currentSetpoint > 0) {
    			currentSetpoint--;
    			
    			disableArmPid();
    			armControl.setSetpoint(setpoints[currentSetpoint]);
    			enableArmPid();
    		}
    	}
    	
    	//lower arm to next setpoint, unless arm is at intake setpoint
    	if(armDownState.updateState(stick1.getRawButton(RTRIGGER)|| stick2.getRawButton(RTRIGGER))) {
    		if(currentSetpoint < setpoints.length-1) {
    			currentSetpoint++;
    			
    			disableArmPid();
    			armControl.setSetpoint(setpoints[currentSetpoint]);
    			enableArmPid();
    		}
    	}
    }
    
    public void runTrigger() {
    	//run the trigger only if the shooter wheel is at speed (or the B button is still held down from a previous run)
    	if((stick1.getRawButton(BBUTTON) || stick2.getRawButton(BBUTTON)) 
    			&& (firingInProgress || (shooterIsAtSpeed(100) && armEncoder.get() < (HIGHTRAVELSETPOINT - OFFSET)))) {
    		
    		trigger.setAngle(TRIGGERONPOSITION);
    		if(!firingInProgress){
    			TalkToPi.rawCommand("WATCH");
    			lastPiMessage = Timer.getFPGATimestamp();
    			firingInProgress = true;
    		}
    		
    	}
    	else {
    		trigger.setAngle(TRIGGEROFFPOSITION);
    		firingInProgress = false;
    	}
    }
    
    public class GyroDriveOutput implements PIDOutput {
    	
		@Override
		public void pidWrite(double output) {
			SmartDashboard.putNumber("Gyro Drive PIDOutput", output);
			goNoDrifting(output, -gyro.getAngle() * SmartDashboard.getNumber("k_angle"), 0.1, 0.5);
			
		}
		
	}
    
    //TODO make this work for just one side
    public class motionDriveOutput implements PIDOutput {
    	
		@Override
		public void pidWrite(double output) {
			SmartDashboard.putNumber("Motion Drive PIDOutput", output);
			lDrive1.set(output);
			lDrive2.set(output);
			rDrive1.set(-output);
			rDrive2.set(-output);
		}
		
	}
    
    public class GyroTurnOutput implements PIDOutput {
    	
		@Override
		public void pidWrite(double output) {
			SmartDashboard.putNumber("Gyro Turn PIDOutput", output);
			lDrive1.set(output);
			lDrive2.set(output);
			rDrive1.set(output);
			rDrive2.set(output);
		}
	}
    public class ArmOutput implements PIDOutput {
    	
  		@Override
  		public void pidWrite(double output) {
  			
  			
  			SmartDashboard.putNumber("Arm PIDOutput", output);
  			arm.set(output);
  		}
  	}
    
    public class AverageEncoder implements PIDSource {
		PIDSource leftSource;
		PIDSource rightSource;
		
		public AverageEncoder(PIDSource left, PIDSource right) {
			leftSource = left;
			rightSource = right;
		}
		
		@Override
		public double pidGet() {
			return driveEncoderToInches((leftSource.pidGet() + rightSource.pidGet()) / 2);
		}
		
		@Override
		public void setPIDSourceType(PIDSourceType pidSource) {
			// TODO Auto-generated method stub
			
		}
		
		@Override
		public PIDSourceType getPIDSourceType() {
			// TODO Auto-generated method stub
			return PIDSourceType.kDisplacement;
		}
	}
    
    public void enableArmPid() {
		
		armControl.enableLog("armPID.csv");
		armControl.enable();
    }
    
    public void disableArmPid() {
		
		armControl.disable();
		//armControl.closeLog();
    }
    
    public double leftIRToDistance() {
		return voltageToDistance(leftIR.getAverageVoltage(),
				LEFTVOLTAGES, LEFTDISTANCES);
	}

	public double rightIRToDistance() {
		return voltageToDistance(rightIR.getAverageVoltage(),
				RIGHTVOLTAGES, RIGHTDISTANCES);
	}

	public double voltageToDistance(double v, double[] voltages, double[] distances) {
		int i = 0;
		while (i < voltages.length && v < voltages[i]) {
			i++;
		}

		if (i == 0) {
			return -1;
		}
		if (i >= voltages.length) {
			return 40;
		}

		double d2 = distances[i];
		double d1 = distances[i - 1];
		double v2 = voltages[i];
		double v1 = voltages[i - 1];
		return (d2 - d1) * (v - v1) / (v2 - v1) + d1;
	}
    
	public static double driveInchesToEncoder(double i) {
		return i * ENCODER_RESOLUTION / (2 * Math.PI * R * DRIVE_RATIO);
	}
	
	public static double driveEncoderToInches(double e) {
		return e * 2 * Math.PI * R * DRIVE_RATIO / (ENCODER_RESOLUTION);
	}
    
    public void goNoDrifting(double outputMagnitude, double curve, double minimum, double sensitivity) {
        double leftOutput, rightOutput;
        
        if (Math.abs(outputMagnitude) < minimum)
        	minimum = Math.abs(outputMagnitude);
        
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
        if(Math.abs(leftOutput)<Math.abs(minimum)) {
        	leftOutput = minimum;
        }
        if(Math.abs(rightOutput)<Math.abs(minimum)) {
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
    	if(armEncoder.get() < (MIDSETPOINT - OFFSET)) {
    		if (gyroDrive) {
    			turnControl.disable();
    			//turnControl.closeLog();
    			turnControl.reset();
    		}
    		gyroDrive = false;
    		return;
    	}
    	
    	if (stick1.getRawButton(LBUMPER)){
    		goNoDrifting(0.7, -gyro.getAngle() * SmartDashboard.getNumber("k_angle"), 0.5, 0.5);
    		if(!gyroDrive){
    			gyroReset();
    			gyroDrive = true;
    			tankDriveEnabled = false;
    			slowDriveEnabled = false;
    		}
    		
    	} 
    	else if(stick1.getRawButton(LTRIGGER)){
    		goNoDrifting(-0.7, gyro.getAngle() * SmartDashboard.getNumber("k_angle"), -0.5, 0.5);
    		if(!gyroDrive){
    			gyroReset();
    			gyroDrive = true;
    			tankDriveEnabled = false;
    			slowDriveEnabled = false;
    		}
    		
    	} else if(gyroDrive) {
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
    				 //System.out.println(inputLine);
    			 }
    			 
    			 in.close();
    			
    		} catch(IOException e) {}
    		
    	} catch(MalformedURLException e) {}
    }
    
    //QUICKCLICK creations
    //create objects to run drive system
    public void createDriveObjects() {
    	lDriveEncoder = new Encoder(0, 1, false, EncodingType.k4X);
    	rDriveEncoder = new Encoder(2, 3, true, EncodingType.k4X);
    	lDriveEncoder.reset();
    	rDriveEncoder.reset();
    	
    	leftIR = new AnalogInput(0);
    	rightIR = new AnalogInput(1);
    	
    	
    	gyro = new ADXRS450_Gyro(Port.kOnboardCS0);
    	
    	//calibrate gyro
    	gyro.calibrate();
    	
    	lDrive1 = new VictorSP(2);
    	lDrive2 = new VictorSP(3);
    	rDrive1 = new VictorSP(0);
    	rDrive2 = new VictorSP(1);
    	
    	turnControl = new ATM2016PIDController(0.08, 0.0000001, 0.005, gyro, new GyroTurnOutput());
    	//visionTurnControl = new ATM2016PIDController(0.08, 0.0000001, 0.005, gyro, new GyroTurnOutput()); 
    	visionTurnControl = new ATM2016PIDController(0.04, 0.0000001, 0.08, gyro, new GyroTurnOutput());
    	
    	//gyroDriveControl = new ATM2016PIDController  (0.05, 0.00015, 0.05, new AverageEncoder(lDriveEncoder, rDriveEncoder), new GyroDriveOutput(), 0.01);
    	//motionDriveControl = new ATM2016PIDController(0.05, 0.00015, 0.05, new AverageEncoder(lDriveEncoder, rDriveEncoder), new motionDriveOutput(), 0.01);
    	gyroDriveControl = new ATM2016PIDController  (0.07, 0.00015, 0.2, new AverageEncoder(lDriveEncoder, rDriveEncoder), new GyroDriveOutput(), 0.01);
    	motionDriveControl = new ATM2016PIDController(0.07, 0.00025, 0.2, new AverageEncoder(lDriveEncoder, rDriveEncoder), new motionDriveOutput(), 0.01);
    	

    	motionDriveControl.setKaKv(0.0027, 0.0079);
    	//motionDriveControl.setKaKv(0.002, 0.01087);
    	gyroDriveControl.setKaKv(0.002, 0.01087);
    	
    	emergencyState = new ToggleSwitch();
    }
    
    //create objects to run intake system
    public void createIntakeObjects() {
    	intake = new Talon(4);
    	
    	intakeInState = new ToggleSwitch();
    	intakeOutState = new ToggleSwitch();
    	intakeOffState = new ToggleSwitch();
    	
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
    	RPMState = new ToggleSwitch();
    	visionShootState = new ToggleSwitch();
    }
    
    //create objects to run the arm system
    public void createArmObjects() {
    	upperLimitSwitch = new DigitalInput(8);
    	lowerLimitSwitch = new DigitalInput(7);
    	
    	arm = new CANTalon(0);
    	
    	armEncoder = new Encoder(5, 6, true, EncodingType.k4X);
    	armEncoder.reset();
    	//armEncoder = new CANPIDSource(arm);
    	armControl = new ATM2016PIDController(0.004, 0.000075, 0.075, 0.0, armEncoder, arm, 0.01);
    	armControl.setRobot(this);
    	
    	armUpState = new ToggleSwitch();
    	armDownState = new ToggleSwitch();
    	encoderResetState = new ToggleSwitch();
    	
    }
    
    //put initial values to the SmartDashboard
    public void putSmartDashboardValues() {

    	SmartDashboard.putNumber("P", 0.05);
    	SmartDashboard.putNumber("I", 0.00015);
    	SmartDashboard.putNumber("D", 0.05);
    	SmartDashboard.putNumber("F", 0.0);
    	SmartDashboard.putNumber("k_angle", 0.1);
    	SmartDashboard.putNumber("k_sensitivity", 0.5);
    	SmartDashboard.putNumber("Arm Speed", 0.0);
    	SmartDashboard.putNumber("TestGyro Target (Inches)", 0);
    	SmartDashboard.putNumber("GyroDrive Initial Power", 0.0);
    	SmartDashboard.putNumber("GyroDrive Initial Time (ms)", 0.0);
    	SmartDashboard.putNumber("Arm Target", 0);
    	SmartDashboard.putNumber("Motion Plan Target", 0.0);
    	SmartDashboard.putNumber("TestDrive Power", 0.0);
    	SmartDashboard.putNumber("TargetShooterSpeed", 0.0);
    	//SmartDashboard.putNumber("Servo Angle", TRIGGEROFFPOSITION);
    	SmartDashboard.putBoolean("Shoot in Auto?", true);
    		
    }
    
    //create autoChooser and add auto modes
    /**WARNING! If any entries are added to/removed from the SendableChooser, the robot AND the driver station computer must be simultaneously rebooted!
     * This comes from the NetworkTables being stored on both the driver station and the roboRIO
     * **/
    
    public void createAutoModes() {
    	
    	autoChooser = new SendableChooser();
    	
		autoChooser.addDefault("Empty: Do Nothing", new EmptyAuto(this));
		autoChooser.addObject("Defense 1 (Low Bar)", new MainAuto(this, 1));
		autoChooser.addObject("Defense 2", new MainAuto(this, 2));
		autoChooser.addObject("Defense 3", new MainAuto(this, 3));
		autoChooser.addObject("Defense 4", new MainAuto(this, 4));
		autoChooser.addObject("Defense 5", new MainAuto(this, 5));
		
    	SmartDashboard.putData("Autonomous Mode", autoChooser);
		
    }
    
    //create objects to run the trigger system
    public void createTriggerObjects() {
    	trigger = new Servo(6);
    	trigger.setAngle(TRIGGEROFFPOSITION);
    }
    
 }
