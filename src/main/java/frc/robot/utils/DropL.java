// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Random;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DropL extends Command {
  /** Creates a new Intake. */
  private Pose3d endPose;

  private Pose3d startPose;
  private double duration;
  private Timer timer = new Timer();
  private static Random random = new Random();

  public DropL() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startPose =
        AllianceFlipUtil.apply(
            new Pose3d(
                0.75,
                7.57,
                1.15,
                new Rotation3d(
                    Units.degreesToRadians(0),
                    Units.degreesToRadians(35),
                    Units.degreesToRadians(-55))));
    endPose =
        AllianceFlipUtil.apply(
            new Pose3d(
                random.nextDouble(1, 2.25),
                random.nextDouble(5, 7),
                Units.inchesToMeters(4.5 / 2),
                new Rotation3d(
                    Units.degreesToRadians(0),
                    Units.degreesToRadians(0),
                    Units.degreesToRadians(random.nextDouble(-55 - 90, -55 + 90)))));
    duration = startPose.getTranslation().getDistance(endPose.getTranslation()) / 16;
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SimCoral.setPose(1, startPose.interpolate(endPose, timer.get() / duration));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(duration);
  }
}
