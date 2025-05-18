package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.SimCoral;
import java.util.Map;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs;

  private ElevatorPosition currentMode = ElevatorPosition.IDLE;

  public Elevator(ElevatorIO io) {
    this.io = io;
    this.inputs = new ElevatorIOInputsAutoLogged();
    SimCoral.start();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  private void setDistance(Distance distance) {
    io.setDistance(distance);
  }

  public enum ElevatorPosition {
    IDLE(Inches.of(0), Inches.of(1)),
    NET(Inches.of(66), Inches.of(1.5)),
    L1(Inches.of(6), Inches.of(1.5)),
    L2(Inches.of(4), Inches.of(1.5)),
    L3(Inches.of(23), Inches.of(1.5)),
    L4(Inches.of(50), Inches.of(1.5));

    private final Distance targetDistance;
    private final Distance distanceTolerance;

    ElevatorPosition(Distance targetDistance, Distance distanceTolerance) {
      this.targetDistance = targetDistance;
      this.distanceTolerance = distanceTolerance;
    }

    ElevatorPosition(Distance targetDistance) {
      this(targetDistance, Inches.of(2.5));
    }
  }

  private void setElevatorPosition(ElevatorPosition mode) {
    if (currentMode != mode) {
      currentCommand.cancel();
      currentMode = mode;
      currentCommand.schedule();
    }
  }

  @AutoLogOutput
  public ElevatorPosition getMode() {
    return currentMode;
  }

  private final Command currentCommand =
      new SelectCommand<>(
          Map.of(
              ElevatorPosition.IDLE,
              createPositionCommand(ElevatorPosition.IDLE),
              ElevatorPosition.NET,
              createPositionCommand(ElevatorPosition.NET),
              ElevatorPosition.L1,
              createPositionCommand(ElevatorPosition.L1),
              ElevatorPosition.L2,
              createPositionCommand(ElevatorPosition.L2),
              ElevatorPosition.L3,
              createPositionCommand(ElevatorPosition.L3),
              ElevatorPosition.L4,
              createPositionCommand(ElevatorPosition.L4)),
          this::getMode);

  private Command createPositionCommand(ElevatorPosition position) {
    return Commands.runOnce(() -> setDistance(position.targetDistance));
  }

  @AutoLogOutput
  public boolean isAtTarget() {
    return inputs.distance.isNear(currentMode.targetDistance, currentMode.distanceTolerance);
  }

  private Command setPositionCommand(ElevatorPosition distance) {
    return Commands.runOnce(() -> setElevatorPosition(distance));
  }

  public final Command IDLE() {
    return setPositionCommand(ElevatorPosition.IDLE);
  }

  public final Command NET() {
    return setPositionCommand(ElevatorPosition.NET);
  }

  public final Command L1() {
    return setPositionCommand(ElevatorPosition.L1);
  }

  public final Command L2() {
    return setPositionCommand(ElevatorPosition.L2);
  }

  public final Command L3() {
    return setPositionCommand(ElevatorPosition.L3);
  }

  public final Command L4() {
    return setPositionCommand(ElevatorPosition.L4);
  }
}
