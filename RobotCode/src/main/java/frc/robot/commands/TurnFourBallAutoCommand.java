// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
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
public class TurnFourBallAutoCommand extends SequentialCommandGroup {
  /** Creates a new FourBallAutoCommand. */

  private final double kP = .055;

  public TurnFourBallAutoCommand(IntakeSubsystem intake, DrivetrainSubsystem drive, TurretSubsystem turret, TowerSubsystem tower,
                            ShooterSubsystem shooter, StopperSubsystem stopper, LimelightSubsystem limelight, ClimberSubsystem climber) {

      // Add your commands in the addCommands() call, e.g.
      // addCommands(new FooCommand(), new BarCommand());

      //PULL DOWN INTAKE
      //PUT CLIMBER DOWN
      //ACTIVATE INTAKE
      //START TOWER AND SHOOTER
      //MOVE 1.6 METERS
      //AUTO AIM
      //RELEASE STOPPER
      //ENGAGE STOPPER
      //MOVE 4.2 METERS
      //MOVE BACK 4.2 METERS
      //AUTO AIM
      //RELEASE STOPPER
      //STOP EVERYTHING
      

      addCommands(new InstantCommand(() -> intake.intakeOut()),
          new InstantCommand(() ->  climber.climberForward()),
          new InstantCommand(() -> {
            intake.runIntake(1.0);
            tower.runTower(-1.0);
          }),
          new InstantCommand(() -> {
            shooter.setShooterVelocity(6200);
          }),
          //new WaitCommand(0.5),
          new InstantCommand(() -> drive.resetEncoders()),
          new AutoStraightMoveCommand(drive, -1.6, drive.getAngle(), new PIDController(kP, 0, 0),.5),
         // new WaitCommand(0.5), // testing
          new AutoAimCommand(turret, limelight),
          new InstantCommand(() -> stopper.stopperIn()),
          new WaitCommand(1.0),
          new InstantCommand(() -> stopper.stopperOut()),
          new InstantCommand(() -> {
            shooter.setShooterVelocity(7100);
            }),
          new AutoTurnCommand(drive,.4,drive.getAngle()+16.0),
          new InstantCommand(() -> drive.resetEncoders()),
          new AutoStraightMoveCommand(drive, -3.8, drive.getAngle(), new PIDController(kP, 0, 0)),
          //new WaitCommand(.5),
          new InstantCommand(() -> drive.resetEncoders()),
          new AutoStraightMoveCommand(drive, 2.75, drive.getAngle(), new PIDController(kP, 0, 0),.5),
          new AutoAimCommand(turret, limelight),
          new InstantCommand(() -> stopper.stopperIn()),
          new WaitCommand(1.0),
          new InstantCommand(() -> {
            shooter.setShooterVelocity(ShooterConstants.IDLE_SPEED);
            tower.runTower(0.0);
            stopper.stopperOut();
          } ),
          new InstantCommand(() -> intake.runIntake(0.0)));
      }
}
