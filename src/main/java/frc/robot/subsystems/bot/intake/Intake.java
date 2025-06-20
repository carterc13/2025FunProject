package frc.robot.subsystems.bot.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs;

  private IntakePosition currentMode = IntakePosition.STOW;

  public Intake(IntakeIO io) {
    this.io = io;
    this.inputs = new IntakeIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("States/INTAKE", currentMode);
  }

  private void setAngle(Angle angle) {
    io.setAngle(angle);
  }

  public enum IntakePosition {
    INTAKE(Degrees.of(-45), Degrees.of(2.5)),
    HANDOFF(Degrees.of(100), Degrees.of(1)),
    STOW(Degrees.of(210), Degrees.of(2.5));

    private final Angle targetAngle;
    private final Angle angleTolerance;

    IntakePosition(Angle targetAngle, Angle angleTolerance) {
      this.targetAngle = targetAngle;
      this.angleTolerance = angleTolerance;
    }

    IntakePosition(Angle targetAngle) {
      this(targetAngle, Degrees.of(2.5));
    }
  }

  private void setIntakePosition(IntakePosition mode) {
    if (currentMode != mode) {
      currentCommand.cancel();
      currentMode = mode;
      currentCommand.schedule();
    }
  }

  @AutoLogOutput
  public IntakePosition getMode() {
    return currentMode;
  }

  private final Command currentCommand =
      new SelectCommand<>(
          Map.of(
              IntakePosition.INTAKE,
              createPositionCommand(IntakePosition.INTAKE),
              IntakePosition.HANDOFF,
              createPositionCommand(IntakePosition.HANDOFF),
              IntakePosition.STOW,
              createPositionCommand(IntakePosition.STOW)),
          this::getMode);

  private Command createPositionCommand(IntakePosition position) {
    return Commands.runOnce(() -> setAngle(position.targetAngle));
  }

  @AutoLogOutput
  public boolean isAtTarget() {
    return inputs.angle.isNear(currentMode.targetAngle, currentMode.angleTolerance);
  }

  private Command setPositionCommand(IntakePosition angle) {
    return Commands.runOnce(() -> setIntakePosition(angle));
  }

  public final Command INTAKE() {
    return setPositionCommand(IntakePosition.INTAKE);
  }

  public final Command HANDOFF() {
    return setPositionCommand(IntakePosition.HANDOFF);
  }

  public final Command STOW() {
    return setPositionCommand(IntakePosition.STOW);
  }

  public final void INTAKEm() {
    setIntakePosition(IntakePosition.INTAKE);
  }

  public final void HANDOFFm() {
    setIntakePosition(IntakePosition.HANDOFF);
  }

  public final void STOWm() {
    setIntakePosition(IntakePosition.STOW);
  }
}
