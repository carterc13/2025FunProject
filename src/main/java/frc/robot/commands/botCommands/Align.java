// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.botCommands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.bot.Memory;
import frc.robot.subsystems.bot.arm.Arm;
import frc.robot.subsystems.bot.arm.Arm.ArmPosition;
import frc.robot.subsystems.bot.elevator.Elevator;
import frc.robot.subsystems.bot.intake.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Align extends Command {
  LinearVelocity MaxSpeed = TunerConstants.kSpeedAt12Volts;
  SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed.times(0.1))
          .withRotationalDeadband(Constants.MaxAngularRate.times(0.1))
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  Arm arm;
  Elevator elevator;
  Intake intake;
  Memory memory;

  /** Creates a new Align. */
  public Align(Arm arm, Elevator elevator, Intake intake, Memory memory) {
    this.arm = arm;
    this.elevator = elevator;
    this.intake = intake;
    this.memory = memory;
    addRequirements(arm, elevator, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (memory.getCurrentTarget().getElevatorPosition()) {
      case L2:
        if (arm.getMode() != ArmPosition.L2) {
          arm.L2().schedule();
          elevator.L2().schedule();
        }
        break;
      case L3:
        if (arm.getMode() != ArmPosition.L3) {
          arm.L3().schedule();
          elevator.L3().schedule();
        }
        break;
      case L4:
        if (arm.getMode() != ArmPosition.L4) {
          arm.L4().schedule();
          elevator.L4().schedule();
        }
        break;
      default:
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // drivetrain
    //     .applyRequest(
    //         () ->
    //             drive
    //                 .withVelocityX(
    //                     MaxSpeed.times(
    //                         -translation.calculate(
    //                             drivetrain.getPose().getX(),
    //                             drivetrain.getReefPosition().getPose().getX())))
    //                 .withVelocityY(
    //                     MaxSpeed.times(
    //                         -translation.calculate(
    //                             drivetrain.getPose().getY(),
    //                             drivetrain.getReefPosition().getPose().getY())))
    //                 .withRotationalRate(
    //                     Constants.MaxAngularRate.times(
    //                         rotation.calculate(
    //                             drivetrain
    //                                 .getPose()
    //                                 .getRotation()
    //                                 .minus(
    //                                     drivetrain
    //                                         .getReefPosition()
    //                                         .getPose()
    //                                         .getRotation()
    //                                         .toRotation2d())
    //                                 .minus(new Rotation2d(Degrees.of(180)))
    //                                 .getDegrees(),
    //                             0))))
    //     .schedule();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
