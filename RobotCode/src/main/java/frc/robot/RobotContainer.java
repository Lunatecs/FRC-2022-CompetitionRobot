// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.sql.Driver;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.commands.ClimbToPosition;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.RamseteCommandFactory;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TowerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final ClimberSubsystem climber = new ClimberSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final TowerSubsystem tower = new TowerSubsystem();

  private final Joystick driverJoystick = new Joystick(Constants.JoystickConstants.DRIVER_USB);
  private final Joystick operatorJoystick = new Joystick(Constants.JoystickConstants.OPERATOR_USB);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    configureDefaultCommands();
  }

  public void configureDefaultCommands() {
   /*
    CommandScheduler.getInstance().setDefaultCommand(drivetrainSubsystem, new ArcadeDriveCommand(drivetrainSubsystem,
                                                     () -> {return driverJoystick.getRawAxis(Constants.JoystickConstants.LEFT_Y_AXIS);}, 
                                                     () -> {return driverJoystick.getRawAxis(Constants.JoystickConstants.RIGHT_X_AXIS);}));
    */
    drivetrainSubsystem.setDefaultCommand(new ArcadeDriveCommand(drivetrainSubsystem,
    () -> {return driverJoystick.getRawAxis(Constants.JoystickConstants.LEFT_Y_AXIS) * .6 ;}, 
    () -> {return driverJoystick.getRawAxis(Constants.JoystickConstants.RIGHT_X_AXIS) * .6  ;}));
    
    intake.setDefaultCommand(new RunCommand( () -> {
      if (driverJoystick.getRawAxis(Constants.JoystickConstants.LEFT_TRIGGER) > 0){
        intake.runIntake(driverJoystick.getRawAxis(Constants.JoystickConstants.LEFT_TRIGGER));
      } else if (driverJoystick.getRawAxis(Constants.JoystickConstants.RIGHT_TRIGGER)> 0){
        intake.runIntake(-driverJoystick.getRawAxis(Constants.JoystickConstants.RIGHT_TRIGGER));
      } else {
        intake.runIntake(0);
      }
    }
    , intake));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new POVButton(operatorJoystick, 0).whileHeld(new RunCommand(() -> climber.runClimber(-.5),climber))
                                      .whenReleased(new RunCommand(() -> climber.runClimber(0),climber));
    new POVButton(operatorJoystick, 180).whileHeld(new RunCommand(() -> climber.runClimber(.5),climber))
                                        .whenReleased(new RunCommand(() -> climber.runClimber(0),climber));

    
    new JoystickButton(operatorJoystick, JoystickConstants.YELLOW_BUTTON).whileHeld(() -> tower.runTower(.5))
                                                                          .whenReleased(() -> tower.runTower(0));
    new JoystickButton(operatorJoystick, JoystickConstants.GREEN_BUTTON).whileHeld(() -> tower.runTower(-.5))
                                                                          .whenReleased(() -> tower.runTower(0));
    new JoystickButton(operatorJoystick, JoystickConstants.RED_BUTTON).whenPressed(() -> climber.toggleSolenoid());

    new JoystickButton(operatorJoystick, JoystickConstants.LEFT_BUMPER).whenPressed(new ClimbToPosition(-156000, climber));
    new JoystickButton(operatorJoystick, JoystickConstants.RIGHT_BUMPER).whenPressed(new ClimbToPosition(0, climber));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return RamseteCommandFactory.getRamseteCommand(drivetrainSubsystem);
  }
}
