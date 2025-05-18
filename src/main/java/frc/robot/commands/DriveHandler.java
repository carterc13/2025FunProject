// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveHandler extends Command {
  LinearVelocity MaxSpeed = TunerConstants.kSpeedAt12Volts;
  SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
			.withDeadband(MaxSpeed.times(0.1))
			.withRotationalDeadband(Constants.MaxAngularRate.times(0.1)) // Add a 10% deadband
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  Drive drivetrain;
  DoubleSupplier ly;
  DoubleSupplier lx;
  DoubleSupplier rx;

  public DriveHandler(Drive drivetrain, DoubleSupplier ly, DoubleSupplier lx, DoubleSupplier rx) {
    this.drivetrain = drivetrain;
    this.ly = ly;
    this.lx = lx;
    this.rx = rx;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.applyRequest(
        () -> drive
            .withVelocityX(MaxSpeed.times(ly.getAsDouble()))
            .withVelocityY(MaxSpeed.times(lx.getAsDouble()))
            .withRotationalRate(
                Constants.MaxAngularRate.times(rx.getAsDouble()))).schedule();
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
