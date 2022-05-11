// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.sql.Driver;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.BallSensorConstants;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.buttons.TriggerButton;
import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.AutoMoveCommand;
import frc.robot.commands.AutoTurnCommand;
import frc.robot.commands.ClimbToPosition;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.FourBallAutoCommand;
import frc.robot.commands.ManualTurret;
import frc.robot.commands.NewFourBallAutoCommand;
import frc.robot.commands.NewOneBallAutoCommand;
import frc.robot.commands.OneBallAutoCommand;
import frc.robot.commands.OneButtonShootCommand;
import frc.robot.commands.RamseteCommandFactory;
import frc.robot.commands.ZeroTurretCommand;
import frc.robot.subsystems.BallSensorSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StopperSubsystem;
import frc.robot.subsystems.TimerSubsystem;
import frc.robot.subsystems.TowerSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.ScanForTargetCommand;
import frc.robot.commands.TurnFourBallAutoCommand;
import frc.robot.commands.TwoBallAutoCommand;

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
  private final TurretSubsystem turret = new TurretSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final LimelightSubsystem limelight = new LimelightSubsystem();
  private final StopperSubsystem stopper = new StopperSubsystem();
  private final BallSensorSubsystem ballSensor = new BallSensorSubsystem();
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();
  private final TimerSubsystem timer = new TimerSubsystem();

  private final Joystick driverJoystick = new Joystick(Constants.JoystickConstants.DRIVER_USB);
  private final Joystick operatorJoystick = new Joystick(Constants.JoystickConstants.OPERATOR_USB);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    LEDSubsystem.setLEDSubsystem(ledSubsystem);
    CameraServer.startAutomaticCapture();
    configureButtonBindings();
    configureDefaultCommands();
    configureAutos();
    
  }

  public void configureAutos() {
    //autoChooser.setDefaultOption("4 Ball Auto",new FourBallAutoCommand(intake, drivetrainSubsystem, turret, tower, shooter, stopper, limelight));
    autoChooser.setDefaultOption("2 Ball Auto", new TwoBallAutoCommand(intake, drivetrainSubsystem, turret, tower, shooter, stopper, limelight, climber));
    autoChooser.addOption("New 1 Ball Auto", new NewOneBallAutoCommand(intake, drivetrainSubsystem, turret, tower, shooter, stopper, limelight, climber));
    autoChooser.addOption("New 4 Ball Auto",new NewFourBallAutoCommand(intake, drivetrainSubsystem, turret, tower, shooter, stopper, limelight, climber));
    autoChooser.addOption("Turn 4 Ball Auto",new TurnFourBallAutoCommand(intake, drivetrainSubsystem, turret, tower, shooter, stopper, limelight, climber));
    autoChooser.addOption("1 Ball Auto", new OneBallAutoCommand(intake, drivetrainSubsystem, turret, tower, shooter, stopper, limelight));
    autoChooser.addOption("Move", new AutoMoveCommand(drivetrainSubsystem, 1.5));
    autoChooser.addOption("Do Nothing", new InstantCommand());
    autoChooser.addOption("Test Auto Turn", new AutoTurnCommand(drivetrainSubsystem, .5, drivetrainSubsystem.getAngle()+10));
    SmartDashboard.putData(autoChooser);
  }

  public void configureDefaultCommands() {
   /*
    CommandScheduler.getInstance().setDefaultCommand(drivetrainSubsystem, new ArcadeDriveCommand(drivetrainSubsystem,
                                                     () -> {return driverJoystick.getRawAxis(Constants.JoystickConstants.LEFT_Y_AXIS);}, 
                                                     () -> {return driverJoystick.getRawAxis(Constants.JoystickConstants.RIGHT_X_AXIS);}));
    */
    drivetrainSubsystem.setDefaultCommand(new ArcadeDriveCommand(drivetrainSubsystem,
    () -> {return driverJoystick.getRawAxis(Constants.JoystickConstants.LEFT_Y_AXIS);}, 
    () -> {return driverJoystick.getRawAxis(Constants.JoystickConstants.RIGHT_X_AXIS);},
    () -> {return driverJoystick.getRawButton(JoystickConstants.YELLOW_BUTTON);},
    () -> {return driverJoystick.getRawButton(JoystickConstants.LEFT_BUMPER);}));
    /*
    intake.setDefaultCommand(new RunCommand( () -> {
      if (driverJoystick.getRawAxis(Constants.JoystickConstants.LEFT_TRIGGER) > 0){
        intake.runIntake(driverJoystick.getRawAxis(Constants.JoystickConstants.LEFT_TRIGGER)*0.75);
        tower.runTower(-1.0);
      } else if (driverJoystick.getRawAxis(Constants.JoystickConstants.RIGHT_TRIGGER)> 0){
        intake.runIntake(-driverJoystick.getRawAxis(Constants.JoystickConstants.RIGHT_TRIGGER)*0.75);
        tower.runTower(1.0);
      } else {
        intake.runIntake(0);
        tower.runTower(0);
      }
    }
    , intake, tower));
*/
    turret.setDefaultCommand(new RunCommand( () -> {
      if( operatorJoystick.getRawAxis(Constants.JoystickConstants.LEFT_X_AXIS) != 0) {
        turret.setTurretSpeed(operatorJoystick.getRawAxis(Constants.JoystickConstants.LEFT_X_AXIS), true);
      } else {
        turret.setTurretSpeed(0, true);
      }
    }, turret));
    /*
    shooter.setDefaultCommand(new RunCommand(() -> {
      shooter.setShooterSpeed(-operatorJoystick.getRawAxis(JoystickConstants.RIGHT_Y_AXIS));
    }, shooter));
    */
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /*
   
    new POVButton(operatorJoystick, 0).whenPressed(new RunCommand( ()-> {shooter.setShooterVelocity(5800);
                                                                         tower.runTower(-1.0);},shooter, tower),true)
                                      .whenReleased(new RunCommand( () -> {shooter.setShooterVelocity(0);
                                                                           tower.runTower(0.0);},shooter, tower),true);

   new POVButton(operatorJoystick, 180).whenPressed(new RunCommand( () -> {shooter.setShooterVelocity(7500);
                                                                            tower.runTower(-1.0);},shooter, tower),true)
                                      .whenReleased(new RunCommand( () -> {shooter.setShooterVelocity(0);
                                                                           tower.runTower(0.0);}, shooter, tower),true);
   
    */                                                                        
    new POVButton(operatorJoystick, 0).whenPressed(new OneButtonShootCommand(5800, shooter, stopper, tower))
                                      .whenReleased(new RunCommand( () -> {shooter.setShooterVelocity(ShooterConstants.IDLE_SPEED);
                                                                          tower.runTower(0.0);
                                                                          stopper.stopperOut();},shooter, tower, stopper),true);
                                        
    new POVButton(operatorJoystick, 90).whenPressed(new OneButtonShootCommand(6400, shooter, stopper, tower))
                                      .whenReleased(new RunCommand( () -> {shooter.setShooterVelocity(ShooterConstants.IDLE_SPEED);
                                                                          tower.runTower(0.0);
                                                                          stopper.stopperOut();},shooter, tower, stopper),true);

    new POVButton(operatorJoystick, 180).whenPressed(new OneButtonShootCommand(7100, shooter, stopper, tower))
                                      .whenReleased(new RunCommand( () -> {shooter.setShooterVelocity(ShooterConstants.IDLE_SPEED);
                                                                          tower.runTower(0.0);
                                                                          stopper.stopperOut();},shooter, tower, stopper),true);

    new POVButton(operatorJoystick, 270).whenPressed(new OneButtonShootCommand(7700, shooter, stopper, tower))
                                      .whenReleased(new RunCommand( () -> {shooter.setShooterVelocity(ShooterConstants.IDLE_SPEED);
                                                                          tower.runTower(0.0);
                                                                          stopper.stopperOut();},shooter, tower, stopper),true);
     /*
    We could potentially replace the operator control over the tower intake and outtake by instead having the tower
    bound to the intake.
    */
    new JoystickButton(operatorJoystick, JoystickConstants.YELLOW_BUTTON).whileHeld(new RunCommand(() -> climber.runClimber(.75), climber))
                                                                          .whenReleased(new   RunCommand(() -> climber.runClimber(0), climber));  
    new JoystickButton(operatorJoystick, JoystickConstants.GREEN_BUTTON).whileHeld(new RunCommand(() -> climber.runClimber(-.5), climber))
                                                                          .whenReleased(new   RunCommand(() -> climber.runClimber(0), climber));                      

    new TriggerButton(() -> operatorJoystick.getRawAxis(JoystickConstants.RIGHT_TRIGGER)).and(new JoystickButton(operatorJoystick, JoystickConstants.RED_BUTTON)).whenActive(() -> climber.toggleSolenoid());

    new JoystickButton(operatorJoystick, JoystickConstants.LEFT_BUMPER).whenPressed(new ClimbToPosition(230000, climber));
    //new JoystickButton(operatorJoystick, JoystickConstants.RIGHT_BUMPER).whenPressed(new ClimbToPosition(0, climber));

    new TriggerButton(() -> operatorJoystick.getRawAxis(JoystickConstants.LEFT_TRIGGER)).whileActiveContinuous(new ManualTurret(() -> operatorJoystick.getRawAxis(JoystickConstants.LEFT_X_AXIS), turret));
    
    new JoystickButton(operatorJoystick, JoystickConstants.BLUE_BUTTON).whenPressed(new ScanForTargetCommand(turret, limelight));

    new JoystickButton(operatorJoystick,JoystickConstants.RIGHT_BUMPER).whenPressed(new InstantCommand( () -> stopper.stopperIn(),stopper)).whenReleased(new InstantCommand(() -> stopper.stopperOut(),stopper));

    new JoystickButton(driverJoystick, JoystickConstants.GREEN_BUTTON).whenPressed( new RunCommand(
          () -> {
            intake.runIntake(.3);
            tower.runTower(-1.0);
          },
          intake, tower
        ), true).whenReleased(new InstantCommand(
          () -> {
            intake.runIntake(0);
            tower.runTower(0);
          },
          intake, tower
        ), true);

    new JoystickButton(driverJoystick, JoystickConstants.RED_BUTTON).whenPressed(() -> intake.toggleFrontIntake());
    new JoystickButton(driverJoystick, JoystickConstants.BLUE_BUTTON).whenPressed(() -> intake.toggleBackIntake());
    new JoystickButton(driverJoystick, JoystickConstants.START_BUTTON).whenPressed(new ZeroTurretCommand(turret));
    new JoystickButton(driverJoystick, JoystickConstants.BACK_BUTTON).whenPressed(()-> {climber.toggleArms();
                                                                                        ledSubsystem.addColor(ledSubsystem.CLIMB);});
    
    new TriggerButton(() -> driverJoystick.getRawAxis(JoystickConstants.LEFT_TRIGGER)).whenActive(new RunCommand(
      () -> {
        intake.runIntake(driverJoystick.getRawAxis(Constants.JoystickConstants.LEFT_TRIGGER));
        tower.runTower(-1.0);
      },
      intake, tower
    ), true).whenInactive(new InstantCommand(
      () -> {
        intake.runIntake(0);
        tower.runTower(0);
      },
      intake, tower
    ), true);
    new TriggerButton(() -> driverJoystick.getRawAxis(JoystickConstants.RIGHT_TRIGGER)).whenActive(new RunCommand(
      () -> {
        intake.runIntake(-driverJoystick.getRawAxis(Constants.JoystickConstants.RIGHT_TRIGGER));
        tower.runTower(1.0);
      },
      intake, tower
    ), true).whenInactive(new InstantCommand(
      () -> {
        intake.runIntake(0);
        tower.runTower(0);
      },
      intake, tower
    ), true);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return new AutoMoveCommand(drivetrainSubsystem, -1.5);
    //return new OneBallAutoCommand(intake, drivetrainSubsystem, turret, tower, shooter, stopper, limelight);
    //return new TwoBallAutoCommand(intake, drivetrainSubsystem, turret, tower, shooter, stopper, limelight);
    //return new FourBallAutoCommand(intake, drivetrainSubsystem, turret, tower, shooter, stopper, limelight);
    return autoChooser.getSelected();
    //return RamseteCommandFactory.getRamseteCommand(drivetrainSubsystem);
  }
}
