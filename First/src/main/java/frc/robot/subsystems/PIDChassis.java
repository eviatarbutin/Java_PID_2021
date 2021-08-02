// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/*
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

import static frc.robot.Constants.Chassis.*;

public class PIDChassis extends PIDSubsystem {
  /** Creates a new PIDChassis. /
  private WPI_TalonSRX rightMaster= new WPI_TalonSRX(RIGHT_MASTER_PORT);
	private WPI_TalonSRX rightSlave= new WPI_TalonSRX(RIGHT_SLAVE_PORT);
	private WPI_TalonSRX leftMaster= new WPI_TalonSRX(LEFT_MASTER_PORT);
  private WPI_TalonSRX leftSlave= new WPI_TalonSRX(LEFT_SLAVE_PORT);
  
	private DifferentialDrive differentialDrive= new DifferentialDrive(leftMaster, rightMaster); 
	private Encoder encoder= new Encoder(0,1,false,EncodingType.k1X);

  public PIDChassis() {
    super(
        // The PIDController used by the subsystem
        new PIDController(kP, kI, kD));
        getController().setTolerance(tolerance);
        getController().setSetpoint(target);
        encoder.setDistancePerPulse(distancePerPulse);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    arcadeDrive(output, rotationSpeed);
  }

  public void arcadeDrive(final double forwardSpeed, final double rotationSpeed) {
    differentialDrive.arcadeDrive(forwardSpeed, rotationSpeed);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return encoder.getRate();
  }

  public boolean atSetpoint() {
    return getController().atSetpoint();
  }
}
*/