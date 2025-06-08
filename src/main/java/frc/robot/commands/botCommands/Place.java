// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.botCommands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.bot.Memory;
import frc.robot.subsystems.bot.arm.Arm;
import frc.robot.subsystems.bot.drive.Drive;
import frc.robot.subsystems.bot.elevator.Elevator;
import frc.robot.subsystems.bot.intake.Intake;
import frc.robot.utils.SimCoral;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Place extends Command {
  LinearVelocity MaxSpeed = TunerConstants.kSpeedAt12Volts;
  SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed.times(0.1))
          .withRotationalDeadband(Constants.MaxAngularRate.times(0.1))
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  Drive drivetrain;
  Arm arm;
  Elevator elevator;
  Intake intake;
  Memory memory;
  PIDController translation = new PIDController(1, 0, 0.001);
  PIDController rotation = new PIDController(0.04, 0, 0.00075);

  /** Creates a new Align. */
  public Place(Drive drivetrain, Arm arm, Elevator elevator, Intake intake, Memory memory) {
    this.drivetrain = drivetrain;
    this.arm = arm;
    this.elevator = elevator;
    this.intake = intake;
    this.memory = memory;
    addRequirements(drivetrain, arm, elevator, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.PREPINTAKE().schedule();
    elevator.IDLE().schedule();
    intake.HANDOFF().schedule();
    memory.place();
    SimCoral.placeCoral(elevator.getMode(), memory.getCurrentTarget().getReefPosition());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
