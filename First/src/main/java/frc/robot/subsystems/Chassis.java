// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Chassis.*;

public class Chassis extends SubsystemBase {
	/** Creates a new ExampleSubsystem. */
	private WPI_TalonSRX rightMaster;
	private WPI_TalonSRX rightSlave;
	private WPI_TalonSRX leftMaster;
	private WPI_TalonSRX leftSlave;
	private DifferentialDrive differentialDrive; 
	private Encoder rightEncoder;
	private Encoder leftEncoder;


	public Chassis() {
		rightMaster = new WPI_TalonSRX(RIGHT_MASTER_PORT);
		rightSlave = new WPI_TalonSRX(RIGHT_SLAVE_PORT);
		rightSlave.follow(rightMaster);

		leftMaster = new WPI_TalonSRX(LEFT_MASTER_PORT);
		leftSlave = new WPI_TalonSRX(LEFT_SLAVE_PORT);
		leftSlave.follow(leftMaster);

		differentialDrive = new DifferentialDrive(leftMaster, rightMaster);

		rightEncoder = new Encoder(RIGHT_ENCODER_PORT[0],RIGHT_ENCODER_PORT[1]);
		leftEncoder = new Encoder(LEFT_ENCODER_PORT[0], LEFT_ENCODER_PORT[1]);

		leftEncoder.setDistancePerPulse(kEncoderDistancePerPulse);
		rightEncoder.setDistancePerPulse(kEncoderDistancePerPulse);
		resetEncoders();
	}

	public void setRightSpeed(final double speed){
		rightMaster.set(speed);
	}

	public void setLeftSpeed(final double speed){
		leftMaster.set(speed);
	}

	public void drive(final double speed, final double turnSpeed){
		differentialDrive.arcadeDrive(speed, turnSpeed);
	}

	public Encoder getRightEncoder(){
		return rightEncoder;
	}

	public Encoder getLeftEncoder(){
		return leftEncoder;
	}

	public void resetEncoders() {
		leftEncoder.reset();
		rightEncoder.reset();
	}
	
	@Override
	public void periodic() {
	// This method will be called once per scheduler run
	}

	@Override
	public void simulationPeriodic() {
	// This method will be called once per scheduler run during simulation
}
}
