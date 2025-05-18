// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.commands.PathfindThenFollowPath;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.State;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveCommand extends Command {
  Drive drivetrain;
  DoubleSupplier lx;
  DoubleSupplier ly;
  DoubleSupplier rx;
  LinearVelocity MaxSpeed = TunerConstants.kSpeedAt12Volts;
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed.times(0.1))
                        .withRotationalDeadband(Constants.MaxAngularRate.times(0.1)) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  /** Creates a new DriveCommand. */
  public DriveCommand(Drive drive, DoubleSupplier ly, DoubleSupplier lx, DoubleSupplier rx) {
    this.drivetrain = drive;
    this.lx = lx;
    this.ly = ly;
    this.rx = rx;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (State.getDriveState()) {
      case IDLE:
        drivetrain.applyRequest(
            () -> drive
                .withVelocityX(
                    MaxSpeed.times(
                        ly.getAsDouble())) // Drive forward with negative Y (forward)
                .withVelocityY(
                    MaxSpeed.times(
                        lx.getAsDouble())) // Drive left with negative X (left)
                .withRotationalRate(
                    Constants.MaxAngularRate.times(
                        rx.getAsDouble()))); // Drive counterclockwise with negative X (left)
        break;
      case AUTOTOCORAL:
        break;
      default:

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
