package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class CANPIDSource implements PIDSource {
	CANTalon m_talon;

	public CANPIDSource(CANTalon talon) {

		m_talon = talon;

	}

	public double pidGet() {
		return m_talon.getEncPosition();
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		// TODO Auto-generated method stub
		return null;
	}
}