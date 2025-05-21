// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive.ReefPositions;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import frc.robot.utils.SimCoral;
import java.util.function.BooleanSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PlaceCoral extends Command {
  ElevatorPosition level;
  ReefPositions position;
  BooleanSupplier isRightSource;
  /** Creates a new PlaceCoral. */
  public PlaceCoral(ElevatorPosition level, ReefPositions position, BooleanSupplier isRightSource) {
    this.level = level;
    this.position = position;
    this.isRightSource = isRightSource;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SimCoral.placeCoral(level, position);
    SimCoral.Drop(isRightSource).schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
