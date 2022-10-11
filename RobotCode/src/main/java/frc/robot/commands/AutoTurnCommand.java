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
public class AutoTurnCommand extends PIDCommand {
  DrivetrainSubsystem drive;
  
  /** Creates a new AutoTurnCommand. */
  public AutoTurnCommand(DrivetrainSubsystem drive, double maxSpeed, double angle) {
    super(
        // The controller that the command will use
        new PIDController(.08, 0, 0),
        // This should return the measurement
        () -> drive.getAngle(),
        // This should return the setpoint (can also be a constant)
        () -> angle,
        // This uses the output
        output -> {
          
          output = maxSpeed*(output/Math.abs(output));
          
          
          SmartDashboard.putNumber("turn power", output);
          drive.arcadeDrive(0, output);
        });
        this.drive = drive;
        addRequirements(drive);
        getController().setTolerance(.5);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

// Returns true when the command should end.
@Override
public boolean isFinished() {
  SmartDashboard.putBoolean("Turn Done", getController().atSetpoint());
  if(getController().atSetpoint()) {
    drive.arcadeDrive(0, 0);
  }
  return getController().atSetpoint();
}
}
