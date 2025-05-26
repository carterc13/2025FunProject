// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utils.AllianceFlipUtil;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PathFind extends Command {
  PathConstraints constraints =
      new PathConstraints(5.72, 14.7, 4.634, Units.degreesToRadians(1136));
  GoalEndState end;
  Command currentcommand;
  Drive drivetrain;
  List<Pair<Translation2d, Translation2d>> obstacles = new ArrayList<>();
  private static Random random = new Random();

  /** Creates a new PathFind. */
  public PathFind(Drive drivetrain) {
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (obstacles.isEmpty()) {
      obstacles.add(
          0,
          new Pair<Translation2d, Translation2d>(
              new Translation2d(
                  SmartDashboard.getNumber("tempTransX1", 0),
                  SmartDashboard.getNumber("tempTransY1", 0)),
              new Translation2d(
                  SmartDashboard.getNumber("tempTransX2", 0),
                  SmartDashboard.getNumber("tempTransY2", 0))));
    }
    end = new GoalEndState(0, new Rotation2d(0));
    Pathfinding.setStartPosition(
        AllianceFlipUtil.apply(AutoBuilder.getCurrentPose().getTranslation()));
    Pathfinding.setDynamicObstacles(
        obstacles, AllianceFlipUtil.apply(AutoBuilder.getCurrentPose().getTranslation()));
    Pathfinding.setGoalPosition(new Translation2d(10, 5));
    currentcommand =
        AutoBuilder.followPath(
            Pathfinding.getCurrentPath(
                new PathConstraints(5.72, 14.7, 4.634, Units.degreesToRadians(1136)),
                new GoalEndState(MetersPerSecond.of(1), new Rotation2d())));
    currentcommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.recordOutput(
        "corner1",
        new Translation2d(
            Meters.of(SmartDashboard.getNumber("tempTransX1", 0)),
            Meters.of(SmartDashboard.getNumber("tempTransY1", 0))));
    Logger.recordOutput(
        "corner2",
        new Translation2d(
            Meters.of(SmartDashboard.getNumber("tempTransX2", 0)),
            Meters.of(SmartDashboard.getNumber("tempTransY2", 0))));
    obstacles.set(
        0,
        new Pair<Translation2d, Translation2d>(
            new Translation2d(
                SmartDashboard.getNumber("tempTransX1", 0),
                SmartDashboard.getNumber("tempTransY1", 0)),
            new Translation2d(
                SmartDashboard.getNumber("tempTransX2", 0),
                SmartDashboard.getNumber("tempTransY2", 0))));
    Pathfinding.setDynamicObstacles(
        obstacles, AllianceFlipUtil.apply(AutoBuilder.getCurrentPose().getTranslation()));
    if (Pathfinding.isNewPathAvailable()) {
      currentcommand.cancel();
      currentcommand =
          AutoBuilder.followPath(
              Pathfinding.getCurrentPath(
                  new PathConstraints(5.72, 14.7, 4.634, Units.degreesToRadians(1136)),
                  new GoalEndState(MetersPerSecond.of(1), new Rotation2d())));
      currentcommand.schedule();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    currentcommand.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // currentcommand.isFinished();
  }
}
