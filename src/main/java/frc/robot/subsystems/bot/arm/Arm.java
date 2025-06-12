package frc.robot.subsystems.bot.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs;

  private ArmPosition currentMode = ArmPosition.IDLE;

  public Arm(ArmIO io) {
    this.io = io;
    this.inputs = new ArmIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
    Logger.recordOutput("States/ARM", currentMode);
  }

  private void setAngle(Angle angle) {
    io.setAngle(angle);
  }

  public enum ArmPosition {
    IDLE(Degrees.of(60), Degrees.of(1.5)),
    ALGAEIDLE(Degrees.of(65), Degrees.of(1.5)),
    PREPINTAKE(Degrees.of(60), Degrees.of(1.5)),
    INTAKE(Degrees.of(50), Degrees.of(1.5)),
    CORALIDLE(Degrees.of(30), Degrees.of(1.5)),
    NET(Degrees.of(110), Degrees.of(1.5)),
    L1(Degrees.of(145), Degrees.of(1.5)),
    L2(Degrees.of(125), Degrees.of(1.5)),
    L3(Degrees.of(115), Degrees.of(1.5)),
    L4(Degrees.of(105), Degrees.of(1.5));

    private final Angle targetAngle;
    private final Angle angleTolerance;

    ArmPosition(Angle targetAngle, Angle angleTolerance) {
      this.targetAngle = targetAngle;
      this.angleTolerance = angleTolerance;
    }

    ArmPosition(Angle targetAngle) {
      this(targetAngle, Degrees.of(2.5));
    }
  }

  private void setArmPosition(ArmPosition mode) {
    if (currentMode != mode) {
      currentCommand.cancel();
      currentMode = mode;
      currentCommand.schedule();
    }
  }

  @AutoLogOutput
  public ArmPosition getMode() {
    return currentMode;
  }

  private final Command currentCommand =
      new SelectCommand<>(
          Map.of(
              ArmPosition.IDLE,
              createPositionCommand(ArmPosition.IDLE),
              ArmPosition.ALGAEIDLE,
              createPositionCommand(ArmPosition.ALGAEIDLE),
              ArmPosition.PREPINTAKE,
              createPositionCommand(ArmPosition.PREPINTAKE),
              ArmPosition.INTAKE,
              createPositionCommand(ArmPosition.INTAKE),
              ArmPosition.CORALIDLE,
              createPositionCommand(ArmPosition.CORALIDLE),
              ArmPosition.NET,
              createPositionCommand(ArmPosition.NET),
              ArmPosition.L1,
              createPositionCommand(ArmPosition.L1),
              ArmPosition.L2,
              createPositionCommand(ArmPosition.L2),
              ArmPosition.L3,
              createPositionCommand(ArmPosition.L3),
              ArmPosition.L4,
              createPositionCommand(ArmPosition.L4)),
          this::getMode);

  private Command createPositionCommand(ArmPosition position) {
    return Commands.runOnce(() -> setAngle(position.targetAngle));
  }

  @AutoLogOutput
  public boolean isAtTarget() {
    return inputs.angle.isNear(currentMode.targetAngle, currentMode.angleTolerance);
  }

  private Command setPositionCommand(ArmPosition angle) {
    return Commands.runOnce(() -> setArmPosition(angle));
  }

  public final Command IDLE() {
    return setPositionCommand(ArmPosition.IDLE);
  }

  public final Command ALGAEIDLE() {
    return setPositionCommand(ArmPosition.ALGAEIDLE);
  }

  public final Command PREPINTAKE() {
    return setPositionCommand(ArmPosition.PREPINTAKE);
  }

  public final Command INTAKE() {
    return setPositionCommand(ArmPosition.INTAKE);
  }

  public final Command CORALIDLE() {
    return setPositionCommand(ArmPosition.CORALIDLE);
  }

  public final Command NET() {
    return setPositionCommand(ArmPosition.NET);
  }

  public final Command L1() {
    return setPositionCommand(ArmPosition.L1);
  }

  public final Command L2() {
    return setPositionCommand(ArmPosition.L2);
  }

  public final Command L3() {
    return setPositionCommand(ArmPosition.L3);
  }

  public final Command L4() {
    return setPositionCommand(ArmPosition.L4);
  }

  public final void IDLEm() {
    setArmPosition(ArmPosition.IDLE);
  }

  public final void ALGAEIDLEm() {
    setArmPosition(ArmPosition.ALGAEIDLE);
  }

  public final void PREPINTAKEm() {
    setArmPosition(ArmPosition.PREPINTAKE);
  }

  public final void INTAKEm() {
    setArmPosition(ArmPosition.INTAKE);
  }

  public final void CORALIDLEm() {
    setArmPosition(ArmPosition.CORALIDLE);
  }

  public final void NETm() {
    setArmPosition(ArmPosition.NET);
  }

  public final void L1m() {
    setArmPosition(ArmPosition.L1);
  }

  public final void L2m() {
    setArmPosition(ArmPosition.L2);
  }

  public final void L3m() {
    setArmPosition(ArmPosition.L3);
  }

  public final void L4m() {
    setArmPosition(ArmPosition.L4);
  }
}
