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
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalInput;

import java.io.*;
import java.net.*;

public class Robot extends IterativeRobot {
	
	//QUICKCLICK declarations
	
	/*declare ~magic~ numbers*/
	static final int SHOOTSETPOINT = 0;
	static final int HIGHTRAVELSETPOINT = 600;
	static final int MIDSETPOINT = 1800;
	static final int LOWTRAVELSETPOINT = 2280;
	static final int INTAKESETPOINT = 2400;
	static final int OFFSET = 100;
	
	static final double FARSPEED = 3400.0;
	static final double MIDSPEED = 3200.0;
	static final double CLOSESPEED = 3900.0;
	
	static final int TRIGGEROFFPOSITION = 110;
	static final int TRIGGERONPOSITION = 80;
	
	static final int XBUTTON = 1;
	static final int ABUTTON = 2;
	static final int BBUTTON = 3;
	static final int YBUTTON = 4;
	static final int LBUMPER = 5;
	static final int RBUMPER = 6;
	static final int LTRIGGER = 7;
	static final int RTRIGGER = 8;
	
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
	ATM2016PIDController gyroDriveControl;
	ATM2016PIDController motionDriveControl;
	
	double leftSpeed;
	double rightSpeed;
	boolean gyroDrive = false;
	boolean drivePIDEnabled = false;
	boolean motionDriveEnabled = false;
	boolean tankDriveEnabled = true;
	boolean slowDriveEnabled = false;
	
	static final double ENCODER_RESOLUTION = 2048;
	static final double FUDGE_FACTOR = (194.0/196.0);
	static final double R = (5.875 * FUDGE_FACTOR);
	//static final double DRIVE_RATIO = (1.786 / 4.032);
	static final double DRIVE_RATIO = (1.432 / 3.826);
	
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
	
	ToggleSwitch RPMState;
	
	double shooterSpeed = 0.0;
	int currentTargetRPM = 1;
	double[] shooterTargetRPMs = {FARSPEED, MIDSPEED, CLOSESPEED};
	String[] shooterTargetNames = {"Far Away", "Mid", "CLose Up"};
	
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
	boolean manualArmEnabled = true;
	
	/*declare trigger-related objects and variables*/
	Servo trigger;
	
	boolean watchSent = false;
	
	/*declare auto-related objects*/
	SendableChooser autoChooser;
	
	/*declare test mode-related objects*/
	TestMode testMode;
	
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
    }
    
    public void autonomousInit() {
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
    	
    	//reset gyro
    	gyroReset();
    	
    	//disable autonomous PID
    	gyroDriveControl.disable();
    	gyroDriveControl.setOutputRange(-1.0, 1.0);
    
    }
    
    //QUICKCLICK teleopPeriodic
    public void teleopPeriodic() {
    	
    	//run the trigger only if the shooter wheel is at speed
    	if((stick1.getRawButton(BBUTTON) || stick2.getRawButton(BBUTTON)) 
    			&& (true || (shooterIsAtSpeed() && armEncoder.get() < (HIGHTRAVELSETPOINT - OFFSET)))) {
    		//FIXME restore this before competition!
    		//FIXME
    		//FIXME !!!!!!!!!!!!!!!!!!
    		
    		trigger.setAngle(TRIGGERONPOSITION);
    		if(!watchSent){
    			TalkToPi.rawCommand("WATCH");
    			watchSent = true;
    		}
    		
    	}
    	else {
    		trigger.setAngle(TRIGGEROFFPOSITION);
    		watchSent = false;
    	}
    	
    	//Y button
    	/*if(stick.getRawButton(y)) {
    		if(!drivePIDEnabled) {
    			drivePIDEnabled = true;
    			tankDriveEnabled = false;
    			
    			lDriveEncoder.reset();
    			rDriveEncoder.reset();
    			
    			gyroDriveControl.enableLog("gyroDrivePID.csv");
    			gyroDriveControl.setInitialPower(SmartDashboard.getNumber("GyroDrive Initial Power"), SmartDashboard.getNumber("GyroDrive Initial Time (ms)"));
    			gyroDriveControl.enable();
    			gyroDriveControl.setSetpoint(SmartDashboard.getNumber("Gyro Drive Target"));
    		}
    	} else {
    		if(drivePIDEnabled) {
    			drivePIDEnabled = false;
    			tankDriveEnabled = true;
    			
    			gyroDriveControl.disable();
    			gyroDriveControl.closeLog();
    		}
    	}*/
    	
    	/*
    	//use Y button to enable/disable arm PID
    	if(stick.getRawButton(y)) {
    		if(manualArmEnabled) {
    			//turn PID on and drive to setpoint
    			System.out.println("Arm PID should be on");
    			armControl.enableLog("armPID.csv");
    			armControl.enable();
    			armControl.setSetpoint(SmartDashboard.getNumber("Arm Target"));
    			
    			manualArmEnabled = false;
    		}
    	} else {
    		if(!manualArmEnabled) {
    			System.out.println("Arm PID should be off");
    			armControl.disable();
    			armControl.reset();
    			armControl.closeLog();
    			
    			manualArmEnabled = true;
    		}
    	}*/
    	    	
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
    	
    	/*if(stick.getRawButton(YBUTTON) && !manualArmEnabled) {
    		disableArmPid();
    		armControl.closeLog();
    		
    	} else if(manualArmEnabled) {
    		manualArmEnabled = false;
    	}*/
    	
    	//FIXME remove true and false before competition!!!!!!!!
    	if (!gyroDrive && !motionDriveEnabled) {
    		if(false && armEncoder.get() < (MIDSETPOINT - OFFSET) && tankDriveEnabled) {
    			tankDriveEnabled = false;
    			slowDriveEnabled = true;
    			
    		} else if(true || (armEncoder.get() > (MIDSETPOINT - OFFSET) && !tankDriveEnabled)) {
    			tankDriveEnabled = true;
    			slowDriveEnabled = false;
    		}
    	}
    	
    	//calculate motor speeds
    	setIntakeSpeed();
    	setShooterSpeed();
    	
    	//if manual drive of the arm is allowed, set the arm speed
    	if(manualArmEnabled) {
    		setArmSpeed();
        	arm.set(armSpeed);  
    	} else {
    		
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
    	
    	if(tankDriveEnabled) {
    		driveRobot(leftSpeed, rightSpeed);
    	} else if(slowDriveEnabled) {
    		driveRobot(leftSpeed * 0.2, rightSpeed * 0.2);
    	}
    	
    	//send data to Smart Dashboard
    	/**
    	 * TgtShtrSpeed: 3900 up close (0 robot widths)
    	 * TgtShtrSpeed: 3200 @ 1 robot width
    	 * TgtShtrSpeed: 3300 @ 2 robot widths
    	 * TgtShtrSpeed: 3400 far away (3 robot widths)
    	 */
    	SmartDashboard.putString("ShooterTargetRPM", shooterTargetRPMs[currentTargetRPM] + "( " + shooterTargetNames[currentTargetRPM] + " )");
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
    	
    	if(shooterSpeedControl.isEnabled()) {
    		SmartDashboard.putNumber("Error", shooterSpeedControl.getAvgError());
    	}
    	
    	//update PID constants to Smart Dashboard values
    	//turnControl.setPID(SmartDashboard.getNumber("P"), SmartDashboard.getNumber("I"), SmartDashboard.getNumber("D"));
    	//armControl.setPID(SmartDashboard.getNumber("P"), SmartDashboard.getNumber("I"), SmartDashboard.getNumber("D"));
    	//gyroDriveControl.setPID(SmartDashboard.getNumber("P"), SmartDashboard.getNumber("I"), SmartDashboard.getNumber("D"), SmartDashboard.getNumber("F"));
    	motionDriveControl.setPID(SmartDashboard.getNumber("P"), SmartDashboard.getNumber("I"), SmartDashboard.getNumber("D"));
    	
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
    
    public boolean shooterIsAtSpeed() {
    	double actualSpeed = Math.abs(shooterCounter.getRateInRPMs());
    	double targetSpeed = Math.abs(SmartDashboard.getNumber("TargetShooterSpeed"));
    	
    	if(Math.abs(targetSpeed - actualSpeed) < 250) {
    		return true;
    	}
    	
    	return false;
    }
    
    //QUICKCLICK set motor speeds
    public void setIntakeSpeed() {
    	//set intake using left trigger and left bumper
    	if(intakeState.updateState(stick1.getRawButton(LBUMPER))) {
    		System.out.println("Updated button");
    		if(intakeState.switchEnabled()) {
    			intakeSpeed = -1.0;
    			intakeOn = "In";
    			
    		} else {
    			intakeSpeed = 1.0;
        		intakeOn = "Out";
    		}
    	}
    	
    	if(intakeEnableState.updateState(stick1.getRawButton(LTRIGGER)) ) {
    		turnIntakeOff();
    		
    	}
    }
    
    public void turnIntakeOff(){
		intakeSpeed = 0.0;
		intakeOn = "Off";
		intakeState.reset();
    }
    
    public void setShooterSpeed() {
    	//if the X button is pressed, use PID to drive shooter wheel
    	if(pidState.updateState(stick1.getRawButton(XBUTTON))) {
    		System.out.println("Updated button");
    		if(pidState.switchEnabled()) {
    			System.out.println("Shooter PID should be on");
    			shooterSpeedControl.enableLog("ShootPID.csv");
    			shooterSpeedControl.enable();
    			shooterSpeedControl.setSetpointInRPMs(shooterTargetRPMs[currentTargetRPM]);
    			
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
    	}
    }
    
    public void setArmSpeed() {
    	//set the arm speed using the left and right bumper
    	if(stick1.getRawButton(RBUMPER) && !upperLimitSwitch.get()){
    		armSpeed = -SmartDashboard.getNumber("Arm Speed");
    	}
    	else if(stick1.getRawButton(RTRIGGER)/* && !lowerLimitSwitch.get()*/){
    		armSpeed = SmartDashboard.getNumber("Arm Speed");
    	}
    	else {
    		armSpeed = 0.0;
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
		manualArmEnabled = false;
		
		armControl.enableLog("armPID.csv");
		armControl.enable();
    }
    
    public void disableArmPid() {
		manualArmEnabled = true;
		
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
    	//FIXME remove false before competition!!!!!!!!!!!!!!!!!!11!11!1!11!!
    	if(false && armEncoder.get() < (MIDSETPOINT - OFFSET)) {
    		if (gyroDrive) {
    			turnControl.disable();
    			//turnControl.closeLog();
    			turnControl.reset();
    		}
    		gyroDrive = false;
    		return;
    	}
    	if((stick1.getPOV() >= 0 && stick1.getPOV() <= 45) || (stick1.getPOV()>=315)) {
    		goNoDrifting(0.7, -gyro.getAngle() * SmartDashboard.getNumber("k_angle"), 0.5, 0.5);
    		if(!gyroDrive){
    			gyroReset();
    			gyroDrive = true;
    			tankDriveEnabled = false;
    			slowDriveEnabled = false;
    		}
    		
    	} else if(stick1.getPOV() >= 135 && stick1.getPOV()<= 225) {
    		goNoDrifting(-0.7, gyro.getAngle() * SmartDashboard.getNumber("k_angle"), -0.5, 0.5);
    		if(!gyroDrive){
    			gyroReset();
    			gyroDrive = true;
    			tankDriveEnabled = false;
    			slowDriveEnabled = false;
    		}
    	}else if(stick1.getPOV() == 90) {
    		if(!gyroDrive){
    			gyroReset();
    			turnControl.setSetpoint(90);
    			//turnControl.enableLog("TurnPID.csv");
    			turnControl.enable();
    			
    			gyroDrive = true;
    			tankDriveEnabled = false;
    			slowDriveEnabled = false;
    		}
    	} else if(stick1.getPOV() == 270) {
    		if(!gyroDrive){
    			gyroReset();
    			turnControl.setSetpoint(-90);
    			//turnControl.enableLog("TurnPID.csv");
    			turnControl.enable();
    			
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
    				 System.out.println(inputLine);
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
    	gyroDriveControl = new ATM2016PIDController(0.02, 0.00001, 0.05, new AverageEncoder(lDriveEncoder, rDriveEncoder), new GyroDriveOutput());
    	motionDriveControl = new ATM2016PIDController(0.05, 0.00015, 0.05, new AverageEncoder(lDriveEncoder, rDriveEncoder), new motionDriveOutput(), 0.01);

    	//motionDriveControl.setKaKv(0.0027, 0.0079);
    	motionDriveControl.setKaKv(0.002, 0.01087);
    	
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
    	RPMState = new ToggleSwitch();
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
    	SmartDashboard.putBoolean("Lowbar?", true);
    		
    }
    
    //create autoChooser and add auto modes
    /**WARNING! If any entries are added to/removed from the SendableChooser, the robot AND the driver station computer must be simultaneously rebooted!
     * This comes from the NetworkTables being stored on both the driver station and the roboRIO
     * **/
    
    public void createAutoModes() {
    	
    	autoChooser = new SendableChooser();
    	
		autoChooser.addObject("Empty: Do Nothing", new EmptyAuto(this));
		autoChooser.addObject("Drive Back", new MoveBackAuto(this));
		autoChooser.addObject("Cross Defense", new CrossDefenseAuto(this));
		autoChooser.addObject("Cross Low Bar and Shoot", new CrossAndShootAuto(this));
		autoChooser.addDefault("Calibrate", new CalibrateAuto(this));
		
    	SmartDashboard.putData("Autonomous Mode", autoChooser);
		
    }
    
    //create objects to run the trigger system
    public void createTriggerObjects() {
    	trigger = new Servo(6);
    	trigger.setAngle(TRIGGEROFFPOSITION);
    }
 }