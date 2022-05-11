// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StopperSubsystem;
import frc.robot.subsystems.TowerSubsystem;
import frc.robot.subsystems.TurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NewOneBallAutoCommand extends SequentialCommandGroup {

  IntakeSubsystem intake;
  DrivetrainSubsystem drive;
  TurretSubsystem turret;
  TowerSubsystem tower;
  ShooterSubsystem shooter;
  StopperSubsystem stopper;
  LimelightSubsystem limelight;

  /** Creates a new AutoCommand. */
  public NewOneBallAutoCommand(IntakeSubsystem intake, DrivetrainSubsystem drive, TurretSubsystem turret, TowerSubsystem tower,
                      ShooterSubsystem shooter, StopperSubsystem stopper, LimelightSubsystem limelight, ClimberSubsystem climber) {
    this.intake = intake;
    this.drive = drive;
    this.turret = turret;
    this.tower = tower;
    this.shooter = shooter;
    this.stopper = stopper;
    this.limelight = limelight;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    //PULL DOWN INTAKE
    //ACTIVATE INTAKE
    //MOVE 1.25 METERS
    //AUTO AIM
    //START TOWER AND SHOOTER
    //RELEASE STOPPER
    //STOP EVERYTHING

    addCommands(//new InstantCommand(() -> intake.intakeOut()),
                new WaitCommand(5),
                new InstantCommand(() ->  climber.climberForward()),
                new InstantCommand(() -> {
                  //intake.runIntake(1.0);
                  tower.runTower(-.8);
                }),
                new InstantCommand(() -> {
                  shooter.setShooterVelocity(6000);
                }),
                new WaitCommand(0.5),
                new InstantCommand(() -> drive.resetEncoders()),
                new AutoMoveCommand(drive, -1.25),
                new AutoAimCommand(turret, limelight),
                new InstantCommand(() -> stopper.stopperIn()),
                new WaitCommand(1.5),
                new InstantCommand(() -> {
                  shooter.setShooterVelocity(0);
                  tower.runTower(0.0);
                  stopper.stopperOut();
                } ),
                new InstantCommand(() -> intake.runIntake(0.0)));
  }
}
// add end method