// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StopperSubsystem;
import frc.robot.subsystems.TowerSubsystem;

public class OneButtonShootCommand extends CommandBase {
  /** Creates a new OneButtonShootCommand. */
  private ShooterSubsystem shooter;
  private StopperSubsystem stopper;
  private TowerSubsystem tower;
  private int velocity;
  private int count;

  public OneButtonShootCommand(int velocity, ShooterSubsystem shooter, StopperSubsystem stopper, TowerSubsystem tower) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, stopper, tower);
    this.shooter= shooter;
    this.stopper = stopper;
    this.tower = tower;
    this.velocity = velocity;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setShooterVelocity(velocity);
    tower.runTower(-.8);
    count = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(within(shooter.getShooterVelocity(),velocity, 200.0) && count > 10) {
      stopper.stopperIn();
    } else if(within(shooter.getShooterVelocity(),velocity, 200.0)) {
      count++;
    } else {
      count = 0;
    }
    /*time the stopper going out by counting the amount of ticks*/
  }


  public boolean within(double value, double target, double range) {

    if(value > target - range && value < target + range) {
      return true;
    } else {
      return false;
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setShooterVelocity(ShooterConstants.IDLE_SPEED);
    tower.runTower(0.0);
    stopper.stopperOut();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
