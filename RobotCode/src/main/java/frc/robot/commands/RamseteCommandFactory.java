// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

/** Add your docs here. */
public class RamseteCommandFactory {

    public static RamseteCommand getRamseteCommand(DrivetrainSubsystem drivetrainSubsystem) {
        DifferentialDriveVoltageConstraint differentialDriveVoltageConstraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(DrivetrainConstants.KS, DrivetrainConstants.KV, DrivetrainConstants.KA), 
            DrivetrainConstants.DRIVE_KINEMATICS,
            DrivetrainConstants.MAX_VOLTAGE);
        
            TrajectoryConfig trajectoryConfig = new TrajectoryConfig(DrivetrainConstants.MAX_SPEED, DrivetrainConstants.MAX_VOLTAGE);
            trajectoryConfig.setKinematics(DrivetrainConstants.DRIVE_KINEMATICS);
            trajectoryConfig.addConstraint(differentialDriveVoltageConstraint);
            Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                //start at origin facing +x direction
                new Pose2d(0, 0, new Rotation2d(0)),
                //pass through two interior waypoints, making an s-curve path
                List.of(new Translation2d(1,1), new Translation2d(2, -1)),
                //end 3 meters straight ahead of starting point, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                //pass in trajectoryConfig
            trajectoryConfig);

            RamseteCommand ramseteCommand = new RamseteCommand(
                exampleTrajectory, 
                //:: = pass method in
                drivetrainSubsystem::getPose,
                //functionally the same as above:
                //() -> {return drivetrainSubsystem.getPose();},
                new RamseteController(DrivetrainConstants.RAMSETE_B, DrivetrainConstants.RAMSETE_ZETA), 
                new SimpleMotorFeedforward(DrivetrainConstants.KS, DrivetrainConstants.KV, DrivetrainConstants.KA),
                DrivetrainConstants.DRIVE_KINEMATICS,
                drivetrainSubsystem::getWheelSpeeds, 
                //left
                new PIDController(DrivetrainConstants.KP, 0.0, 0.0),
                //right 
                new PIDController(DrivetrainConstants.KP, 0.0, 0.0), 
                drivetrainSubsystem::voltageTankDrive, 
                drivetrainSubsystem);

            return ramseteCommand;
    }

}