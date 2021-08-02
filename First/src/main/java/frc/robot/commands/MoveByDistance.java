// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Chassis;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.Chassis.*;

/** An example command that uses an example subsystem. */
public class MoveByDistance extends CommandBase {
	private final Chassis chassis;
	private final PIDController pidControllerLeft;
	private final PIDController pidControllerRight;
	private double setpointLeft = 10;
	private double setpointRight = 10;
	/**
	 * Creates a new ExampleCommand.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public MoveByDistance(Chassis chassis) {
		this.chassis = chassis;
		addRequirements(this.chassis);

		pidControllerRight = new PIDController(kP, kI, kD);
		pidControllerLeft = new PIDController(kP, kI, kD);


		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		chassis.resetEncoders();
		
		pidControllerRight.setSetpoint(target);
		pidControllerRight.setTolerance(tolerance);
		
		pidControllerLeft.setSetpoint(target);
		pidControllerLeft.setTolerance(tolerance);
}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		chassis.setRightSpeed(pidControllerRight.calculate(chassis.getRightEncoder().getDistance(),setpointRight));
		chassis.setLeftSpeed(pidControllerLeft.calculate(chassis.getLeftEncoder().getDistance(),setpointLeft));
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		chassis.setRightSpeed(0);
		chassis.setLeftSpeed(0);
		chassis.resetEncoders();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return pidControllerRight.atSetpoint() && pidControllerLeft.atSetpoint();
	}
}
