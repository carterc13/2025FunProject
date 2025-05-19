// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.State;
import frc.robot.State.GamePieceStates;
import frc.robot.State.ReefPositions;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import frc.robot.utils.SimCoral;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PlaceCoral extends Command {
  ElevatorPosition level;
  ReefPositions position;
  /** Creates a new PlaceCoral. */
  public PlaceCoral(ElevatorPosition level, ReefPositions position) {
    this.level = level;
    this.position = position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (State.getGamePieceState() == GamePieceStates.CORAL) {
      SimCoral.placeCoral(level, position);
      SimCoral.Drop().schedule();
      State.setGamePieceState(GamePieceStates.NONE);
    }
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
