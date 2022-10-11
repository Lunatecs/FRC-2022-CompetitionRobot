// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ClimberSubsystem;


public class ClimbToPosition extends CommandBase {
  private ClimberSubsystem climberSubsystem;
  private PIDController leftPIDController;
  private PIDController rightPIDController;
  private int position;
  private final double MINIMUM_SPEED = 0.0; //half of .5
  private boolean finished = false;

  /** Creates a new ClimbToPosition. */
  public ClimbToPosition(int position, ClimberSubsystem climberSubsystem) {
    addRequirements(climberSubsystem);
    this.climberSubsystem = climberSubsystem;
    this.position = position;

    leftPIDController = new PIDController(0.0001, 0.0, 0.0);
    rightPIDController = new PIDController(0.0001, 0.0, 0.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leftPIDController.setSetpoint(position);
    rightPIDController.setSetpoint(position);

    leftPIDController.setTolerance(1000);
    rightPIDController.setTolerance(1000);

    this.finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftSpeed = leftPIDController.calculate(climberSubsystem.getLeftClimberEncoder());
    double rightSpeed = rightPIDController.calculate(climberSubsystem.getRightClimberEncoder());

    if(Math.abs(leftSpeed) < MINIMUM_SPEED) {
      leftSpeed = leftSpeed/(Math.abs(leftSpeed))*MINIMUM_SPEED;
    }

    if(Math.abs(rightSpeed) < MINIMUM_SPEED) {
      rightSpeed = rightSpeed/(Math.abs(rightSpeed))*MINIMUM_SPEED;
    }

    if(leftPIDController.atSetpoint()) {
      leftSpeed = 0;
    }
    if(rightPIDController.atSetpoint()) {
      rightSpeed = 0;
    }
    climberSubsystem.runClimberLeft(leftSpeed);
    climberSubsystem.runClimberRight(rightSpeed);

    //SmartDashboard.putNumber("Left Speed", leftSpeed);
    //SmartDashboard.putNumber("Right Speed", rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climberSubsystem.runClimber(0);
    finished = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
