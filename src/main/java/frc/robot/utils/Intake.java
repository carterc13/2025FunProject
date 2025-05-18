// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Intake extends Command {
  /** Creates a new Intake. */
  private Pose3d endPose;

  private Pose2d pose;

  private Pose3d startPose;
  private double duration;
  private Timer timer = new Timer();

  public Intake(DoubleSupplier posex, DoubleSupplier posey, DoubleSupplier poserotation) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pose =
        new Pose2d(
            posex.getAsDouble(), posey.getAsDouble(), new Rotation2d(poserotation.getAsDouble()));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startPose =
        new Pose3d(
            0.75,
            PoseComputer.isRightSource(pose) ? 0.5 : 7.57,
            1.15,
            new Rotation3d(
                Units.degreesToRadians(0),
                Units.degreesToRadians(35),
                Units.degreesToRadians(PoseComputer.isRightSource(pose) ? 55 : -55)));
    endPose =
        new Pose3d(
          PoseComputer.isRightSource(pose) ? (Math.random() * (2.5 - 1 + 1) + 1) : (Math.random() * (2.5 - 1 + 1) + 1),
          PoseComputer.isRightSource(pose) ? (Math.random() * (4 - 1.5 + 1) + 1.5) : (Math.random() * (7.5 - 4 + 1) + 4),
            0.0,
            new Rotation3d(
                Units.degreesToRadians(0),
                Units.degreesToRadians(0),
                Units.degreesToRadians(Math.random() * 360)));
    duration = startPose.getTranslation().getDistance(endPose.getTranslation()) / 2;
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SimCoral.setPose(0, startPose.interpolate(endPose, timer.get() / duration));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SimCoral.setPose(0, new Pose3d());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(duration);
  }
}
