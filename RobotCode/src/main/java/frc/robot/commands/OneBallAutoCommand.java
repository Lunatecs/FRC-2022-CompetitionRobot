// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
public class OneBallAutoCommand extends SequentialCommandGroup {
  /** Creates a new OneBallAutoCommand. */
  public OneBallAutoCommand(IntakeSubsystem intake, DrivetrainSubsystem drive, TurretSubsystem turret, TowerSubsystem tower,
  ShooterSubsystem shooter, StopperSubsystem stopper, LimelightSubsystem limelight) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    //PULL DOWN INTAKE
    //ACTIVATE INTAKE
    //MOVE 3 FEET
    //AUTO AIM
    //START TOWER AND SHOOTER
    //RELEASE STOPPER

    addCommands(new AutoMoveCommand(drive, -1.25),
                new AutoAimCommand(turret, limelight),
                new InstantCommand(() -> {
                  shooter.setShooterVelocity(6700);
                  tower.runTower(-1.0);
                }),
                new WaitCommand(2.0),
                new InstantCommand(() -> stopper.toggleStopper()),
                new WaitCommand(2.0),
                new InstantCommand(() -> {
                  shooter.setShooterVelocity(0);
                  tower.runTower(0.0);
                  stopper.stopperOut();
                } ));
  }
}
