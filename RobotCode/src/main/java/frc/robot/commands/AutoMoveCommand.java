// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoMoveCommand extends PIDCommand {
  /** Creates a new AutoMoveCommand. */

  DrivetrainSubsystem drive;


  public AutoMoveCommand(DrivetrainSubsystem drive, double meters) {
    super(
        // The controller that the command will use
        new PIDController(5, 0, 0),
        // This should return the measurement
        () -> drive.getLeftDistance(),
        // This should return the setpoint (can also be a constant)
        () -> meters,
        // This uses the output
        output -> {
          //SmartDashboard.putNumber("auto move", output);
          if(Math.abs(output)>.65) {
            output = output/Math.abs(output) * .65;
          }
          drive.arcadeDrive(output, 0);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    this.drive = drive;
    getController().setTolerance(.05);
    addRequirements(drive);
    drive.resetEncoders();
  }

  public AutoMoveCommand(DrivetrainSubsystem drive, double meters, double maxSpeed) {
    super(
        // The controller that the command will use
        new PIDController(5, 0, 0),
        // This should return the measurement
        () -> drive.getLeftDistance(),
        // This should return the setpoint (can also be a constant)
        () -> meters,
        // This uses the output
        output -> {
          //SmartDashboard.putNumber("auto move", output);
          if(Math.abs(output)>maxSpeed) {
            output = output/Math.abs(output) * maxSpeed;
          }
          drive.arcadeDrive(output, 0);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    this.drive = drive;
    getController().setTolerance(.05);
    addRequirements(drive);
    drive.resetEncoders();
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
