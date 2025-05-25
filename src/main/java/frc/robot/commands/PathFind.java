// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utils.AllianceFlipUtil;
import java.util.Random;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PathFind extends Command {
  PathConstraints constraints =
      new PathConstraints(1000, 1000, Units.degreesToRadians(540), Units.degreesToRadians(1090));
  GoalEndState end;
  Command currentcommand;
  Drive drivetrain;
  private static Random random = new Random();
  /** Creates a new PathFind. */
  public PathFind(Drive drivetrain) {
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    end = new GoalEndState(0, new Rotation2d(0));
    Pathfinding.setStartPosition(
        AllianceFlipUtil.apply(AutoBuilder.getCurrentPose().getTranslation()));
    Pathfinding.setGoalPosition(new Translation2d(10, 5));
    currentcommand = drivetrain.pathfinderToPose(() -> new Rotation2d(0));
    currentcommand = drivetrain.pathfinderToPose(() -> new Rotation2d());
    // random.nextDouble(0, 10),
    // random.nextDouble(0, 10),
    // new Rotation2d(random.nextDouble(0, 359.9), random.nextDouble(0, 359.9))));
    currentcommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Pathfinding.isNewPathAvailable()) {
      currentcommand.cancel();
      currentcommand = drivetrain.pathfinderToPose(() -> new Rotation2d(0));
      currentcommand.schedule();
    }
    // Logger.recordOutput("OMGPP", Pathfinding.getCurrentPath(constraints,
    // end).getIdealStartingState().velocity());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    currentcommand.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
