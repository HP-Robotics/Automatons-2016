package org.usfirst.frc.team2823.robot;

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
	//this should be 40 for the old arm
	static final int CALIBRATIONOFFSET = -51;
	
	static final int SHOOTSETPOINT = (-75 + CALIBRATIONOFFSET);
	static final int HIGHTRAVELSETPOINT = (600 + CALIBRATIONOFFSET);
	static final int MIDSETPOINT = (1800 + CALIBRATIONOFFSET);
	static final int CHEVALSETPOINT = (2058 + CALIBRATIONOFFSET);
	static final int INTAKESETPOINT = (2400 + CALIBRATIONOFFSET);
	
	static final int OFFSET = 100;
	static final int AUTO_DRIVE_LEFT_OFFSET = 3;
	
	//these values work well for the old flywheel
	//static final double FARSPEED = 3300.0;
	//static final double MIDSPEED = 3200.0;
	//static final double CLOSESPEED = 3425.0;
	
	//these values work well for the new flywheel
	static final double FARSPEED = 3700.0;
	static final double MIDSPEED = 3600.0;
	static final double CLOSESPEED = 3900.0;
	static final double OUTERSPEED = 4025.0;
	
	static final double LEFTGOALDISTANCE = 13.25;
	static final double CAMERAANGLE = 38;
	static final double VISION_DRIVE_OFFSET = -5;
    static final double VISION_DRIVE_ACCURATE_ENOUGH = 1.5;
	static final double THRESHOLD_VISION_ANGLE = 5;
	static final double VISION_WAIT_TIME = 0.5;
	static final double DISTANCE_CORRECT_THRESHOLD = 5.0;
	static final double DISTANCE_CORRECT_FUDGE = 1.0;
	
	static final int TRIGGEROFFPOSITION = 95;
	static final int TRIGGERONPOSITION = 65;
	
	static final double PORTCULLIS_LOW_POWER = 0.1;
	static final double PORTCULLIS_HIGH_POWER = 0.5;
	static final double PORTCULLIS_UP = 1.0;
	static final double PORTCULLIS_DOWN = -1.0;
	
	static final int XBUTTON = 1;
	static final int ABUTTON = 2;
	static final int BBUTTON = 3;
	static final int YBUTTON = 4;
	static final int LBUMPER = 5;
	static final int RBUMPER = 6;
	static final int LTRIGGER = 7;
	static final int RTRIGGER = 8;
	static final int BACKBUTTON = 9;
	static final int STARTBUTTON = 10;
	
	static final int LEFTAXIS = 1;
	static final int RIGHTAXIS = 3;
	
	static final double DRIVETHRESHOLD = 0.05;
	
	static final double MAXACCELERATION = 275;	//inches / sec^2
	static final double MAXVELOCITY = 120;		//inches / sec
	
	static final double k_ANGLE = 0.1;
	
	/*declare drive-related objects and variables*/
	Encoder lDriveEncoder;
	Encoder rDriveEncoder;
	ADXRS450_Gyro gyro;
	
	VictorSP lDrive1;
	VictorSP lDrive2;
	VictorSP rDrive1;
	VictorSP rDrive2;
	
	ATM2016PIDController turnControl;
	ATM2016PIDController gyroDriveControl;
	ATM2016PIDController motionDriveControl;
	ATM2016PIDController leftDriveControl;
	ATM2016PIDController rightDriveControl;
	
	
	double leftSpeed;
	double rightSpeed;
	double preTargetPosition = 0.0;
	boolean gyroDrive = false;
	boolean motionDriveEnabled = false;
	boolean tankDriveEnabled = true;
	boolean slowDriveEnabled = false;
	boolean emergencyMode = false;
	
	static final double ENCODER_RESOLUTION = 2048;
	static final double FUDGE_FACTOR = (194.0/196.0);
	static final double R = (5.875 * FUDGE_FACTOR);
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
	boolean turningWithVision = false;
	boolean capturedFirstFrame = false;
	boolean calculatedShotDistance = false;
	boolean preMoved = false;
	boolean clearedVisionAverage = false;
	boolean atShotPosition = false;
	boolean visionShotInProgress = false;
	boolean waitingToCheck = false;
	boolean checkingShotAccuracy = false;
	double visionShotSpeed = 0.0;
	int currentTargetRPM = 2;
	double[] shooterTargetRPMs = {FARSPEED, MIDSPEED, CLOSESPEED, OUTERSPEED};
	String[] shooterTargetNames = {"Far Away", "Mid", "Close Up", "Outerworks"};
	
	/*declare arm-related objects and variables*/
	CANTalon arm;
	
	Encoder armEncoder;
	ATM2016PIDController armControl;
	
	ToggleSwitch armUpState;
	ToggleSwitch armDownState;
	ToggleSwitch armEmergencyUpState;
	ToggleSwitch armEmergencyDownState;
	
	boolean emergencyArmUp = false;
	boolean emergencyArmDown = false;
	
	double emergencyArmUpTime = 0.0;
	double emergencyArmDownTime = 0.0;
	double armSpeed = 0.0;
	int currentSetpoint = 0;
	int[] setpoints = {SHOOTSETPOINT, HIGHTRAVELSETPOINT, MIDSETPOINT, CHEVALSETPOINT, INTAKESETPOINT};
	String[] setpointNames = {"Shoot", "High Travel", "Mid", "Low Travel", "Intake"};
	String emergencyModeDisplay = "";
	
	/*declare trigger-related objects and variables*/
	Servo trigger;
	
	boolean firingInProgress = false;
	
	/*declare portcullis arm related objects and variables*/
	CANTalon portcullisArm;
	
	ToggleSwitch portcullisUpState;
	ToggleSwitch portcullisDownState;
	
	double portcullisInitTime = 0.0;
	double portcullisArmSpeed = 0.0;
	double portcullisDirection = PORTCULLIS_UP;
	double portcullisIntervalTime = 0.0;
	boolean loweredPtcSpeed = false;
	
	/*declare auto-related objects*/
	SendableChooser autoChooser;
	SendableChooser visionAutoChooser;
	SendableChooser portChevalChooser;
	
	public static enum Defense {PORTCULLIS, CHEVAL, OTHER};
	
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
	double checkWaitTime = 0.0;
	double initFrameCaptureTime = 0.0;
	double checkSpeedTime = 0.0;
	
	boolean piIsStarted = false;
	boolean setStopTime = false;
	
	/*declare joystick and button press-related objects*/
	Joystick stick1;
	Joystick stick2;
	
	
	
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
	
	public class DefenseSelector {
		Robot.Defense defense;
		
		public DefenseSelector(Robot.Defense d) {
			defense = d;
		}
		
		public Robot.Defense getDefense() {
			return defense;
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
		
	}*/
	
	
	//QUICKCLICK robotInit
    public void robotInit() {
    	
    	//create objects based on subsystem
    	createDriveObjects();
    	createIntakeObjects();
    	createShooterObjects();
    	createArmObjects();
    	putInitialSmartDashboardValues();
    	createAutoModes();
    	createTriggerObjects();
    	
    	//create test mode
    	testMode = new TestMode(this);
    	
    	//create joystick
    	stick1 = new Joystick(0);
    	stick2 = new Joystick(1);
    	
    	portcullisArm = new CANTalon(1);
    	portcullisUpState = new ToggleSwitch();
    	portcullisDownState = new ToggleSwitch();
    	
    }
    
    //QUICKCLICK autonomousInit
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
    	
    	//begin driving portcullis arm into the robot
    	portcullisArmSpeed = PORTCULLIS_LOW_POWER * PORTCULLIS_UP;
    	
    	if(SmartDashboard.getBoolean("Vision Shot?")) {
    		((AutoMode) visionAutoChooser.getSelected()).autoInit();
    	} else {
    		((AutoMode) autoChooser.getSelected()).autoInit();
    	}
    	
    }
    
    public void autonomousPeriodic() {
    	
    	if(SmartDashboard.getBoolean("Vision Shot?")) {
    		((AutoMode) visionAutoChooser.getSelected()).autoPeriodic();
    	} else {
    		((AutoMode) autoChooser.getSelected()).autoPeriodic();
    	}
    	
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
    	
    	//disable intake and arm motors
    	intakeSpeed = 0.0;
    	armSpeed = 0.0;
    	
    	//begin driving portcullis arm into the robot
    	portcullisArmSpeed = PORTCULLIS_LOW_POWER * PORTCULLIS_UP;
    	
    	//reset encoders
    	lDriveEncoder.reset();
    	rDriveEncoder.reset();
    	
    	//disable arm braking
    	arm.enableBrakeMode(false);
    	arm.enable();
    	
    	//lock arm to current setpoint
    	armControl.enable();
    	
    	//reset gyro
    	gyroReset();
    	
    	//disable autonomous PID
    	gyroDriveControl.reset();
    	gyroDriveControl.setOutputRange(-1.0, 1.0);
    	
    	resetDrivePIDs();
    }
    
    //QUICKCLICK teleopPeriodic
    public void teleopPeriodic() {
    	
    	runTrigger();
    	shootWithMagic();
    	turnWithMagic();
    	
    	//set tank drive or slow drive based on the height of the arm
    	if (!gyroDrive && !motionDriveEnabled) {
    		if(armEncoder.get() < (HIGHTRAVELSETPOINT - OFFSET)) {
    			tankDriveEnabled = false;
    			slowDriveEnabled = true;
    			
    		} else if(armEncoder.get() > (HIGHTRAVELSETPOINT - OFFSET)) {
    			tankDriveEnabled = true;
    			slowDriveEnabled = false;
    		}
    	}
    	
    	//swap into and out of emergency mode when START is pressed
    	if(emergencyState.updateState(stick1.getRawButton(STARTBUTTON) || stick2.getRawButton(STARTBUTTON))) {
    		if(emergencyState.switchEnabled()) {
    			emergencyMode = true;
    			emergencyModeDisplay = "EMERGENCY MODE";
    			
    			disableAndResetArmPid();
    		} else {
    			armControl.setSetpoint(setpoints[currentSetpoint]);
    			armControl.enable();
    			
    			emergencyMode = false;
    			emergencyModeDisplay = "";
    		}
    	}
    	
    	//if manual drive of the arm is allowed, set the arm speed
    	if(emergencyMode) {
    		setArmSpeed();
        	arm.set(armSpeed);  
    	} else {
    		setArmSetpoint();
    	}
    	
    	//calculate motor speeds
    	setIntakeSpeed();
    	setShooterSpeed();
    	setPtcArmSpeed();
    	
    	//calculate drive speeds
    	leftSpeed = (Math.abs(stick1.getRawAxis(LEFTAXIS)) < DRIVETHRESHOLD ? 0.0 : stick1.getRawAxis(LEFTAXIS));
    	rightSpeed = (Math.abs(stick1.getRawAxis(RIGHTAXIS)) < DRIVETHRESHOLD ? 0.0 : stick1.getRawAxis(RIGHTAXIS));
    	
    	leftSpeed = Math.pow(-leftSpeed, 3.0);
    	rightSpeed = Math.pow(-rightSpeed, 3.0);
    	
    	//pulse portcullis motor to prevent janking
    	if(portcullisDirection == PORTCULLIS_UP && Math.abs(Timer.getFPGATimestamp() - portcullisIntervalTime) > 3 && loweredPtcSpeed) {
    		portcullisArmSpeed = PORTCULLIS_UP * PORTCULLIS_HIGH_POWER;
    		
    		if(Math.abs(Timer.getFPGATimestamp() - portcullisIntervalTime) > 3.5) {
    			portcullisIntervalTime = Timer.getFPGATimestamp();
    			portcullisArmSpeed = PORTCULLIS_UP * PORTCULLIS_LOW_POWER;
    		}
    	}
    	
    	//drive motors using calculated speeds
    	intake.set(intakeSpeed);
    	portcullisArm.set(portcullisArmSpeed);
    	
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
    	
    	putPeriodicSmartDashboardValues();
    	
    	//print Pi data to SmartDashboard
    	String[] a;
    	if(pi.getLast() != null) {
    		a = pi.getLast().split(" ");
    	} else {
    		a = new String[5];
    		a[0] = "BAD";
    		a[1] = "0";
    		a[2] = "0";
    		a[3] = "0";
    		a[4] = "0";
    	}
    	
    	SmartDashboard.putString("Is vision good", a[0]);
    	SmartDashboard.putString("Camera-to-goal angle", a[1]);
    	SmartDashboard.putString("Camera-to-goal distance", a[3]);
    	SmartDashboard.putString("Camera-to-left-side distance", a[2]);
    	SmartDashboard.putString("Camera-to-right-side distance", a[4]);
    	
    	if(shooterSpeedControl.isEnabled()) {
    		SmartDashboard.putNumber("Error", shooterSpeedControl.getAvgError());
    	}
    	
    	//update PID constants to Smart Dashboard values
    	//turnControl.setPID(SmartDashboard.getNumber("P"), SmartDashboard.getNumber("I"), SmartDashboard.getNumber("D"));
    	//armControl.setPID(SmartDashboard.getNumber("P"), SmartDashboard.getNumber("I"), SmartDashboard.getNumber("D"));
    	//gyroDriveControl.setPID(SmartDashboard.getNumber("P"), SmartDashboard.getNumber("I"), SmartDashboard.getNumber("D"), SmartDashboard.getNumber("F"));
    	//motionDriveControl.setPID(SmartDashboard.getNumber("P"), SmartDashboard.getNumber("I"), SmartDashboard.getNumber("D"));
    	
    	if((Timer.getFPGATimestamp()-lastPiMessage) > 1){
    		TalkToPi.rawCommand("RPM " + shooterTargetRPMs[currentTargetRPM]);
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
    
    //this method overloads the above method and allows the thresholds to be specified separately
    public boolean shooterIsAtSpeed(double lowerThreshold, double upperThreshold) {
    	double actualSpeed = Math.abs(shooterCounter.getRateInRPMs());
    	double targetSpeed = Math.abs(shooterSpeedControl.getSetpointInRPMs());
    	
    	if(actualSpeed > (targetSpeed - lowerThreshold) && actualSpeed < (targetSpeed + upperThreshold)) {
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
    	if(portcullisUpState.updateState(stick2.getRawButton(LBUMPER))) {
    		portcullisArmSpeed = PORTCULLIS_HIGH_POWER;
    		portcullisDirection = PORTCULLIS_UP;
    		portcullisInitTime = Timer.getFPGATimestamp();
    		
    		loweredPtcSpeed = false;
    		SmartDashboard.putString("Portcullis Arm Is:", "Probably Up");
    		
    	} else if (portcullisDownState.updateState(stick2.getRawButton(LTRIGGER))) {
    		portcullisArmSpeed = -PORTCULLIS_HIGH_POWER;
    		portcullisDirection = PORTCULLIS_DOWN;
    		portcullisInitTime = Timer.getFPGATimestamp();
    		
    		loweredPtcSpeed = false;
    		SmartDashboard.putString("Portcullis Arm Is:", "Probably Down");
    		
    	}
    	
    	//lower the portcullis speed after 3 seconds have passed to prevent motor damage
    	if(Math.abs(Timer.getFPGATimestamp() - portcullisInitTime) > 3 && !loweredPtcSpeed) {
    		portcullisArmSpeed = PORTCULLIS_LOW_POWER * portcullisDirection;
    		
    		loweredPtcSpeed = true;
    		
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


    		} else if(!shootingWithVision) {
    			System.out.println("Shooter PID should be off");
    			shooterSpeedControl.reset();
    			shooterSpeedControl.closeLog();


    		}
    	}

    	if (Timer.getFPGATimestamp() - checkSpeedTime > 0.2) {
    		//automatic control of shooter RPM if the vision code is running and can see a goal
    		checkSpeedTime = Timer.getFPGATimestamp();
    		String piData = pi.getLast();
    		if((piData != null) && piData.contains("GOOD")) {

    			//get data from Pi
    			String[] piDataArray = piData.split(" ");
    			try{
    				cameraToGoalAngle = Double.parseDouble(piDataArray[1]);
    				cameraToLeftEdge = Double.parseDouble(piDataArray[2]);
    				cameraToGoalDistance = Double.parseDouble(piDataArray[3]);
    				cameraToRightEdge = Double.parseDouble(piDataArray[4]);

    				//pre-calculate shooter RPM based on distance to wall
    				double visionShotSpeed = (-0.002342381 * Math.pow(cameraToGoalDistance, 3)) + (0.83275224 * Math.pow(cameraToGoalDistance, 2)) +
    						(-89.22806 * cameraToGoalDistance) + 6549.93;

    				shooterSpeedControl.setSetpointInRPMs(visionShotSpeed);

    				TalkToPi.rawCommand("RPM " + visionShotSpeed);
    				lastPiMessage = Timer.getFPGATimestamp();
    			}
    			catch (Exception e){
    				shooterSpeedControl.setSetpointInRPMs(CLOSESPEED);
    				System.out.println("ERROR PARSING MESSAGE " + piData);

    				TalkToPi.rawCommand("RPM " + CLOSESPEED);
    				lastPiMessage=Timer.getFPGATimestamp();
    			}
    		} 

    	}
    	//if the goal isn't visible, spin to the selected shot speed
    	//manual control of shooter RPM
    	if(RPMState.updateState(stick1.getRawButton(YBUTTON)||stick2.getRawButton(YBUTTON))){
    		currentTargetRPM = (currentTargetRPM + 1) % shooterTargetRPMs.length;
    		shooterSpeedControl.setSetpointInRPMs(shooterTargetRPMs[currentTargetRPM]);
    		TalkToPi.rawCommand("RPM " + shooterTargetRPMs[currentTargetRPM]);
    		lastPiMessage = Timer.getFPGATimestamp();
    	}
    }

    public double shortDistanceCorrect(double in) {
        if (Math.abs(in) < DISTANCE_CORRECT_THRESHOLD) {
        	if (in < 0) {
        		return in - DISTANCE_CORRECT_FUDGE;
        	}
        	else {
        		return in + DISTANCE_CORRECT_FUDGE;
        	}
        }
        return in;
    }
    
    //QUICKCLICK shootWithMagic
    public void shootWithMagic () {
    	//run the *magic* button code to read data from the Pi, drive to correct distance, spin up shooter to correct RPM, then fire
    	if((stick1.getRawButton(ABUTTON) || stick2.getRawButton(ABUTTON))) {
    		if(!shootingWithVision) {
    			shootingWithVision = true;
    			
    			System.err.println("A BUTTON PRESSED");
    			
    			//begin Pi video capture
    			TalkToPi.rawCommand("WATCH6");
    			lastPiMessage = Timer.getFPGATimestamp();
    			
    			initFrameCaptureTime = Timer.getFPGATimestamp();
    			System.out.println("INIT CAPTURE TIME: " + initFrameCaptureTime);
    		}
    		
    		if(Math.abs(Timer.getFPGATimestamp() - initFrameCaptureTime) > 0.2 && shootingWithVision && !capturedFirstFrame) {
    			//wait for 200ms before continuing
    			
    			String piData = pi.getLast();
    			System.out.println("INIT PI DATA: " + piData);
    	    	if(piData == null){
    	    		return;
    	    	}
    			
    			capturedFirstFrame = true;
    			
    			//try to fire if the goal is visible
    			if(piData.contains("GOOD")) {
    				
    				motionDriveEnabled = true;
        			tankDriveEnabled= false;
        			slowDriveEnabled = false;
        			
        			//get data from Pi
    				String[] piDataArray = piData.split(" ");
    			
    				cameraToGoalAngle = Double.parseDouble(piDataArray[1]);
    				cameraToLeftEdge = Double.parseDouble(piDataArray[2]);
    				cameraToGoalDistance = Double.parseDouble(piDataArray[3]);
    				cameraToRightEdge = Double.parseDouble(piDataArray[4]);
    				
    				//pre-calculate shooter RPM based on distance to wall
        			double preVisionShotSpeed = (-0.002342381 * Math.pow(cameraToGoalDistance, 3)) + (0.83275224 * Math.pow(cameraToGoalDistance, 2)) +
        									 (-89.22806 * cameraToGoalDistance) + 6549.93;
        			
        			System.out.println("SHOT SPEED: " + preVisionShotSpeed);
        			//spin up shooter wheel to pre-calculated RPM
        			shooterSpeedControl.reset();
        			shooterSpeedControl.setSetpointInRPMs(preVisionShotSpeed);
        			shooterSpeedControl.enable();
    				
    				//pre-calculate motion plan movement
    				preTargetPosition = (cameraToLeftEdge - ((cameraToLeftEdge - cameraToRightEdge) / 2));
        			//preTargetPosition = (cameraToGoalDistance * Math.cos(Math.toRadians(90 - cameraToGoalAngle)));
        			
    				System.out.println("PRE-TURN TARGET: " + preTargetPosition);
    				
    				//check if robot is within an acceptable angle range, and set wait times accordingly
    				setStopTime = true;
    				
    				gyroReset();

    				//Turn if we need to
    				if(Math.abs(cameraToGoalAngle) > THRESHOLD_VISION_ANGLE) {
    					turnControl.setSetpoint(cameraToGoalAngle);
    					System.err.println("CAMERA-TO-GOAL ANGLE " + cameraToGoalAngle);
    					turnControl.enable();
    					
    					stopTime = Timer.getFPGATimestamp() + 1.3;

    				}else {
    					stopTime = Timer.getFPGATimestamp() + 0.6;

    				}
    			}
    		}
    		
    		//Okay, if a move is called for, do that move now.
    		if(Timer.getFPGATimestamp() > (stopTime - 0.6) && setStopTime && !preMoved) {
    			preMoved = true;
    			
    			turnControl.reset();
    			
    			//pre-move based on data from before turn
    			lDriveEncoder.reset();
    			rDriveEncoder.reset();
    			
    			//double target = (preTargetPosition / Math.cos(Math.toRadians(gyro.getAngle())));
    			double target = preTargetPosition * Math.cos(Math.toRadians(gyro.getAngle())) + cameraToGoalDistance * Math.sin(Math.toRadians(gyro.getAngle()));
    			
    			System.out.println("POST-TURN TURN GYRO " + gyro.getAngle());
    			System.out.println("POST-TURN TARGET: " + target);
				//TODO if the target is less than a threshold value, don't try to move
				leftDriveControl.configureGoal(shortDistanceCorrect(target + VISION_DRIVE_OFFSET), Robot.MAXVELOCITY/3, Robot.MAXACCELERATION/5);
				rightDriveControl.configureGoal(shortDistanceCorrect(target + VISION_DRIVE_OFFSET), Robot.MAXVELOCITY/3, Robot.MAXACCELERATION/5);
				leftDriveControl.enable();
				rightDriveControl.enable();
    			
    		}
    		
    		/*if(Timer.getFPGATimestamp()  > stopTime && setStopTime && !calculatedShotDistance) {
				calculatedShotDistance = true;

				//re-get data from Pi after waiting
				String piData = pi.getLast();
				
				String[] piDataArray = piData.split(" ");
    			
				cameraToGoalAngle = Double.parseDouble(piDataArray[1]);
				cameraToLeftEdge = Double.parseDouble(piDataArray[2]);
				cameraToGoalDistance = Double.parseDouble(piDataArray[3]);
				cameraToRightEdge = Double.parseDouble(piDataArray[4]);
				
				if(Math.abs(cameraToGoalAngle) > THRESHOLD_VISION_ANGLE) {
					System.err.println("The vision data has not updated, quitting");
					return;
				}
				
				//calculate shooter RPM based on distance to wall
    			double visionShotSpeed = (-0.002342381 * Math.pow(cameraToGoalDistance, 3)) + (0.83275224 * Math.pow(cameraToGoalDistance, 2)) +
    									 (-89.22806 * cameraToGoalDistance) + 6549.93;
    			
    			//spin up shooter wheel
    			shooterSpeedControl.reset();
    			shooterSpeedControl.setSetpointInRPMs(visionShotSpeed);
    			shooterSpeedControl.enable();
    			
    			//calculate target for motion profile
				//double targetPosition = cameraToLeftEdge - ((cameraToLeftEdge - cameraToRightEdge) / 2);
				double targetPosition = ((cameraToLeftEdge + cameraToRightEdge) / 2);
    			lDriveEncoder.reset();
    			rDriveEncoder.reset();
    			
				//TODO if the target is less than a threshold value, don't try to move
				//System.out.println(targetPosition + "\t" + (Robot.MAXVELOCITY/3) + "\t" + (Robot.MAXACCELERATION/5));
				motionDriveControl.configureGoal(targetPosition, Robot.MAXVELOCITY/3, Robot.MAXACCELERATION/5);
				System.err.println("TARGET " + (targetPosition));
				motionDriveControl.enable();
			}*/
    		
    		if(leftDriveControl.isPlanFinished() && rightDriveControl.isPlanFinished() && /*calculatedShotDistance*/ preMoved && !atShotPosition) {
    			atShotPosition = true;
    			
    			//clear Pi running average
				TalkToPi.rawCommand("CLEAR");
    			lastPiMessage = Timer.getFPGATimestamp();
    			
    			//raise the arm
    			disableAndResetArmPid();
    			armControl.setSetpoint(SHOOTSETPOINT);
    			enableArmPid();
    		}
    	
            // If the arm is up, and speed is good, wait 200 ms for vision to stabilize
    		if((armEncoder.get() < (SHOOTSETPOINT + OFFSET)) && shooterIsAtSpeed(100) && atShotPosition && !waitingToCheck) {
                checkWaitTime = Timer.getFPGATimestamp();
                waitingToCheck = true;
            }

            // If vision has stabilized, look again.
    		if(Math.abs(Timer.getFPGATimestamp() - checkWaitTime) > 0.4 && waitingToCheck && !checkingShotAccuracy) {
    			checkingShotAccuracy = true;
    			String piData = pi.getLast();
    	    	if(piData.contains("GOOD")) {
        			//get data from Pi
    			    System.out.println("CHECK PI DATA: " + piData);
    				String[] piDataArray = piData.split(" ");
    			
    				cameraToLeftEdge = Double.parseDouble(piDataArray[2]);
    				cameraToRightEdge = Double.parseDouble(piDataArray[4]);

    			    double target = (cameraToLeftEdge - ((cameraToLeftEdge - cameraToRightEdge) / 2));
    			    System.out.println("POST-CHECK TARGET: " + target);
    			    
                    if (Math.abs(target + VISION_DRIVE_OFFSET) > VISION_DRIVE_ACCURATE_ENOUGH) {
				        leftDriveControl.configureGoal(shortDistanceCorrect(target + VISION_DRIVE_OFFSET), Robot.MAXVELOCITY/3, Robot.MAXACCELERATION/5);
				        rightDriveControl.configureGoal(shortDistanceCorrect(target + VISION_DRIVE_OFFSET), Robot.MAXVELOCITY/3, Robot.MAXACCELERATION/5);
	                    lDriveEncoder.reset();
	                    rDriveEncoder.reset();
				        leftDriveControl.enable();
				        rightDriveControl.enable();
				        System.out.println("POST-CHECK CORRECTION: " + (target + VISION_DRIVE_OFFSET));
                    }
                } else {
    			    System.out.println("PI DATA NOT GOOD; FIRING ANYWAY");
                }
            }
    			//wait for 200ms before continuing

    		if(leftDriveControl.isPlanFinished() && rightDriveControl.isPlanFinished() && checkingShotAccuracy && !visionShotInProgress) {
    			visionShotInProgress = true;
    			System.out.println("SHOOTING!");
    			System.out.println("LAST MOVE WAS " + driveEncoderToInches(lDriveEncoder.get()));
    			//try to shoot if the arm is above the high travel setpoint and the shooter is at speed
    			trigger.setAngle(TRIGGERONPOSITION);
    		}
    		
    	} else {
    		//stop the *magic* shooting if the button is released after beginning a vision shot
    		if(shootingWithVision) {
    			shootingWithVision = false;
    			capturedFirstFrame = false;
    			calculatedShotDistance = false;
    			clearedVisionAverage = false;
    			preMoved = false;
    			atShotPosition = false;
    			visionShotInProgress = false;
    			waitingToCheck = false;
    			checkingShotAccuracy = false;
    			motionDriveEnabled = false;
    			setStopTime = false;
    			
    			//reset the trigger
    			trigger.setAngle(TRIGGEROFFPOSITION);
    			
    			//lower the arm to the previous setpoint
    			disableAndResetArmPid();
    			armControl.setSetpoint(setpoints[currentSetpoint]);
    			enableArmPid();
    			
    			//stop disabling shooterSpeedControl after vision shot
    			//shooterSpeedControl.reset();
    			leftDriveControl.reset();
    			rightDriveControl.reset();
				turnControl.reset();
    		}
    	}
    }
    public void turnWithMagic () {
    	//run the *magic* button code to read data from the Pi, drive to correct distance, spin up shooter to correct RPM, then fire
    	if((stick1.getRawButton(BACKBUTTON) || stick2.getRawButton(BACKBUTTON))) {
    		if(!turningWithVision) {
    			turningWithVision = true;
    			
				motionDriveEnabled = true;
    			tankDriveEnabled= false;
    			slowDriveEnabled = false;
    			
    			System.err.println("BACK BUTTON PRESSED");
    			
    			//begin Pi video capture
    			TalkToPi.rawCommand("WATCH6");
    			lastPiMessage = Timer.getFPGATimestamp();
    			//wait for 200ms before continuing
    			
    			String piData = pi.getLast();
    			System.out.println("INIT PI DATA: " + piData);
    	    	if(piData == null){
    	    		return;
    	    	}
    			
    			//try to fire if the goal is visible
    			if(piData.contains("GOOD")) {
        			//get data from Pi
    				String[] piDataArray = piData.split(" ");
    
    				double turnAngle = Double.parseDouble(piDataArray[5]);

    				gyroReset();

    				//Turn if we need to
    				if(Math.abs(turnAngle) > 1.0) {
    					turnControl.setSetpoint(turnAngle);
    					System.err.println("Turn Angle" + turnAngle);
    					turnControl.enable();
    				}
    			}
    		}
    	}else {
    		if(turningWithVision){
    		turnControl.reset();
    		}
    		turningWithVision = false;
    		motionDriveEnabled = false;

    	}
    }
  //set the arm speed without using encoders
    public void setArmSpeed() {
    	//begin running the arm up at a higher speed
    	if(armEmergencyUpState.updateState(stick1.getRawButton(RBUMPER) || stick2.getRawButton(RBUMPER))) {
    		emergencyArmUp = true;
    		emergencyArmDown = false;
    		
    		emergencyArmUpTime = Timer.getFPGATimestamp();
    	}
    	
    	//begin running the arm down at a higher speed
    	if(armEmergencyDownState.updateState(stick1.getRawButton(RTRIGGER) || stick2.getRawButton(RTRIGGER))) {
    		emergencyArmUp = false;
    		emergencyArmDown = true;
    		
    		emergencyArmDownTime = Timer.getFPGATimestamp();
    	}
    	
    	//set the arm speed depending on whether the arm should go up or down
    	if(stick1.getRawButton(RTRIGGER) || stick2.getRawButton(RTRIGGER)) {
    		if(Math.abs(Timer.getFPGATimestamp() - emergencyArmDownTime) > 1.5 && emergencyArmDown) {
    			armSpeed = 0.6;
    		} else {
    			armSpeed = 0.2;
    		}
    		
    	} else if(Math.abs(Timer.getFPGATimestamp() - emergencyArmUpTime) > 1.4 && emergencyArmUp) {
    		armSpeed = -0.1;
    		
    	} else if(emergencyArmUp) {
    		armSpeed = -0.6;
    		
    	} else {
    		emergencyArmUp = false;
    		emergencyArmDown = false;
    		
    		armSpeed = 0.0;
    	}
    }
    
    public void setArmSetpoint() {
    	//raise arm to next setpoint, unless arm is at 90 (shoot setpoint)
    	if(armUpState.updateState(stick1.getRawButton(RBUMPER)|| stick2.getRawButton(RBUMPER))) {
    		if(currentSetpoint > 0) {
    			currentSetpoint--;
    			
    			disableAndResetArmPid();
    			armControl.setSetpoint(setpoints[currentSetpoint]);
    			enableArmPid();
    		}
    	}
    	
    	//lower arm to next setpoint, unless arm is at intake setpoint
    	if(armDownState.updateState(stick1.getRawButton(RTRIGGER)|| stick2.getRawButton(RTRIGGER))) {
    		if(currentSetpoint < setpoints.length-1) {
    			currentSetpoint++;
    			
    			disableAndResetArmPid();
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
    	else if(firingInProgress) {
    		trigger.setAngle(TRIGGEROFFPOSITION);
    		firingInProgress = false;
    	}
    }
    
    public class GyroDriveOutput implements PIDOutput {
    	double m_direction = 1.0;
    	
		@Override
		public void pidWrite(double output) {
			SmartDashboard.putNumber("Gyro Drive PIDOutput", output);
			goNoDrifting(output, gyro.getAngle() * k_ANGLE * -m_direction, 0.1, 0.5);
			
		}
		
		public void setDirection(double direction) {
			m_direction = direction;
		}
		
	}
    
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
    
    public class leftDriveOutput implements PIDOutput {
    	@Override
    	public void pidWrite(double output){
    		lDrive1.set(output);
    		lDrive2.set(output);
    	}
    }
    
    public class rightDriveOutput implements PIDOutput {
    	@Override
    	public void pidWrite(double output){
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
    
    public class DriveEncoder implements PIDSource {
		PIDSource source;
		
		public DriveEncoder(PIDSource s) {
			source = s;
		}
		
		@Override
		public double pidGet() {
			return driveEncoderToInches(source.pidGet());
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
    
    public void disableAndResetArmPid() {
		
		armControl.reset();
		//armControl.closeLog();
    }
    
    public void resetDrivePIDs() {
    	motionDriveControl.reset();
    	gyroDriveControl.reset();
    	leftDriveControl.reset();
    	rightDriveControl.reset();
    	turnControl.reset();
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
    		gyroDrive = false;
    		return;
    	}
    	
    	if (stick1.getRawButton(LBUMPER)){
    		goNoDrifting(0.7, -gyro.getAngle() * k_ANGLE, 0.5, 0.5);
    		if(!gyroDrive){
    			gyroReset();
    			gyroDrive = true;
    			tankDriveEnabled = false;
    			slowDriveEnabled = false;
    		}
    		
    	} 
    	else if(stick1.getRawButton(LTRIGGER)){
    		goNoDrifting(-0.7, gyro.getAngle() * k_ANGLE, -0.5, 0.5);
    		if(!gyroDrive){
    			gyroReset();
    			gyroDrive = true;
    			tankDriveEnabled = false;
    			slowDriveEnabled = false;
    		}
    		
    	} else if(gyroDrive) {
    		gyroDrive = false;
    	}
    }
    
    //QUICKCLICK creations
    //create objects to run drive system
    public void createDriveObjects() {
    	lDriveEncoder = new Encoder(0, 1, false, EncodingType.k4X);
    	rDriveEncoder = new Encoder(2, 3, true, EncodingType.k4X);
    	lDriveEncoder.reset();
    	rDriveEncoder.reset();
    	
    	
    	gyro = new ADXRS450_Gyro(Port.kOnboardCS0);
    	
    	//calibrate gyro
    	gyro.calibrate();
    	
    	lDrive1 = new VictorSP(2);
    	lDrive2 = new VictorSP(3);
    	rDrive1 = new VictorSP(0);
    	rDrive2 = new VictorSP(1);
    	
    	turnControl = new ATM2016PIDController(0.08, 0.0000001, 0.005, gyro, new GyroTurnOutput());
    	
    	//these PID constants work well for turning on wood shop concrete
    	//turnControl = new ATM2016PIDController(0.04, 0.0000001, 0.08, gyro, new GyroTurnOutput());
    	
    	//for main robot
    	//gyroDriveControl = new ATM2016PIDController  (0.05, 0.00015, 0.05, new AverageEncoder(lDriveEncoder, rDriveEncoder), new GyroDriveOutput(), 0.01);
    	//motionDriveControl = new ATM2016PIDController(0.05, 0.00015, 0.05, new AverageEncoder(lDriveEncoder, rDriveEncoder), new motionDriveOutput(), 0.01);
    	
    	//for shadow robot
    	gyroDriveControl = new ATM2016PIDController  (0.07, 0.00015, 0.2, new AverageEncoder(lDriveEncoder, rDriveEncoder), new GyroDriveOutput(), 0.01);
    	motionDriveControl = new ATM2016PIDController(0.07, 0.00025, 0.2, new AverageEncoder(lDriveEncoder, rDriveEncoder), new motionDriveOutput(), 0.01);
    	leftDriveControl = new ATM2016PIDController(0.07, 0.00025, 0.2, new DriveEncoder(lDriveEncoder), new leftDriveOutput(), 0.01);
    	rightDriveControl = new ATM2016PIDController(0.07, 0.00025, 0.2, new DriveEncoder(rDriveEncoder), new rightDriveOutput(), 0.01);
    	
    	//motionDriveControl.setKaKv(0.002, 0.01087);
    	motionDriveControl.setKaKv(0.0027, 0.0079);
    	leftDriveControl.setKaKv(0.0027, 0.0079);
    	rightDriveControl.setKaKv(0.0027, 0.0079);
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
    	
    	arm = new CANTalon(0);
    	
    	armEncoder = new Encoder(5, 6, true, EncodingType.k4X);
    	armEncoder.reset();
    	//armEncoder = new CANPIDSource(arm);
    	armControl = new ATM2016PIDController(0.004, 0.000075, 0.075, 0.0, armEncoder, arm, 0.01);
    	armControl.setRobot(this);
    	
    	armUpState = new ToggleSwitch();
    	armDownState = new ToggleSwitch();
    	armEmergencyUpState = new ToggleSwitch();
    	armEmergencyDownState = new ToggleSwitch();
    }
    
    //put initial values to the SmartDashboard
    public void putInitialSmartDashboardValues() {

    	SmartDashboard.putNumber("P", 0.08);
    	SmartDashboard.putNumber("I", 0.0000001);
    	SmartDashboard.putNumber("D", 0.005);
    	SmartDashboard.putNumber("F", 0.0);
    	SmartDashboard.putNumber("Arm Speed", 0.0);
    	SmartDashboard.putNumber("TestGyro Target (Inches)", 0);
    	SmartDashboard.putNumber("Arm Target", 0);
    	SmartDashboard.putNumber("Motion Plan Target", 0.0);
    	SmartDashboard.putNumber("TestDrive Power", 0.0);
    	SmartDashboard.putNumber("TargetShooterSpeed", 0.0);
    	SmartDashboard.putBoolean("Shoot in Auto?", true);
    	SmartDashboard.putBoolean("Vision Shot?", false);
    	SmartDashboard.putString("Portcullis Arm Is:", "Probably Up");
    	SmartDashboard.putString("Emergency Mode", "");
    		
    }
    
    public void putPeriodicSmartDashboardValues(){
    	//send data to Smart Dashboard
    	SmartDashboard.putString("ShooterTargetRPM", shooterTargetRPMs[currentTargetRPM] + "( " + shooterTargetNames[currentTargetRPM] + " )");
    	SmartDashboard.putString("Arm Setpoint", setpointNames[currentSetpoint] + "( " + setpoints[currentSetpoint]+ " )");
    	SmartDashboard.putNumber("lDrive", lDrive1.getSpeed());
    	SmartDashboard.putNumber("rDrive", rDrive1.getSpeed());
    	SmartDashboard.putNumber("Gyro Rotation", gyro.getAngle());
    	SmartDashboard.putNumber("Left Encoder", lDriveEncoder.get());
    	SmartDashboard.putNumber("Right Encoder", rDriveEncoder.get());
    	SmartDashboard.putNumber("Left Encoder (Inches)", driveEncoderToInches(lDriveEncoder.get()));
		SmartDashboard.putNumber("Right Encoder (Inches)", driveEncoderToInches(rDriveEncoder.get()));
    	SmartDashboard.putNumber("Arm Encoder", armEncoder.get());
    	SmartDashboard.putNumber("Shooter Counter", shooterCounter.get());
    	SmartDashboard.putNumber("ActualShooterSpeed", shooterCounter.getRateInRPMs());
    	SmartDashboard.putString("Intake", intakeOn);
    	SmartDashboard.putString("Arm Setpoint", setpointNames[currentSetpoint]);
    	SmartDashboard.putString("Emergency Mode", emergencyModeDisplay);
    	
    }
    
    //create autoChooser and add auto modes
    /**WARNING! If any entries are added to/removed from the SendableChooser, the robot AND the driver station computer must be simultaneously rebooted!
     * This comes from the NetworkTables being stored on both the driver station and the roboRIO
     * **/
    
    public void createAutoModes() {
    	
    	autoChooser = new SendableChooser();
    	visionAutoChooser = new SendableChooser();
    	portChevalChooser = new SendableChooser();
    	
		autoChooser.addDefault("Empty: Do Nothing", new EmptyAuto(this));
		autoChooser.addObject("Defense 1 (Low Bar)", new MainAuto(this, 1));
		autoChooser.addObject("Defense 2", new MainAuto(this, 2));
		autoChooser.addObject("Defense 3", new MainAuto(this, 3));
		autoChooser.addObject("Defense 4", new MainAuto(this, 4));
		autoChooser.addObject("Defense 5", new MainAuto(this, 5));
		
		//SendableChooser for use with vision autonomous
		visionAutoChooser.addDefault("Empty: Do Nothing", new EmptyAuto(this));
		visionAutoChooser.addObject("Defense 2", new MagicVisionAuto(this, 2));
		visionAutoChooser.addObject("Defense 3", new MagicVisionAuto(this, 3));
		visionAutoChooser.addObject("Defense 4", new MagicVisionAuto(this, 4));
		visionAutoChooser.addObject("Defense 5", new MagicVisionAuto(this, 5));
		
		portChevalChooser.addDefault("Other", new DefenseSelector(Defense.OTHER));
		portChevalChooser.addObject("Portcullis", new DefenseSelector(Defense.PORTCULLIS));
		portChevalChooser.addObject("Cheval de Frise", new DefenseSelector(Defense.CHEVAL));
		
    	SmartDashboard.putData("Autonomous Mode", autoChooser);
    	SmartDashboard.putData("Defense", portChevalChooser);
    	SmartDashboard.putData("Vision Autonomous", visionAutoChooser);
		
    }
    
    //create objects to run the trigger system
    public void createTriggerObjects() {
    	trigger = new Servo(6);
    	trigger.setAngle(TRIGGEROFFPOSITION);
    }
    
 }
