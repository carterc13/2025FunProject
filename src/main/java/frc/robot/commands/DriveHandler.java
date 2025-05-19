// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.State;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveHandler extends Command {
  LinearVelocity MaxSpeed = TunerConstants.kSpeedAt12Volts;
  SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed.times(0.1))
          .withRotationalDeadband(Constants.MaxAngularRate.times(0.1)) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  Drive drivetrain;
  DoubleSupplier ly;
  DoubleSupplier lx;
  DoubleSupplier rx;
  PIDController translation = new PIDController(0, 0, 0);
  PIDController rotation = new PIDController(0, 0, 0);

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
    SmartDashboard.putNumber("transp", translation.getP());
    SmartDashboard.putNumber("transi", translation.getI());
    SmartDashboard.putNumber("transd", translation.getD());
    SmartDashboard.putNumber("rotatp", translation.getP());
    SmartDashboard.putNumber("rotati", translation.getI());
    SmartDashboard.putNumber("rotatd", translation.getD());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    translation.setP(SmartDashboard.getNumber("transp", translation.getP()));
    translation.setI(SmartDashboard.getNumber("transi", translation.getI()));
    translation.setD(SmartDashboard.getNumber("transd", translation.getD()));
    rotation.setP(SmartDashboard.getNumber("rotatp", translation.getP()));
    rotation.setI(SmartDashboard.getNumber("rotati", translation.getI()));
    rotation.setD(SmartDashboard.getNumber("rotatd", translation.getD()));
    Logger.recordOutput("Bruh why wont it work", State.getDriveState().toString());
    switch (State.getDriveState()) {
      case IDLE:
        drivetrain
            .applyRequest(
                () ->
                    drive
                        .withVelocityX(MaxSpeed.times(ly.getAsDouble()))
                        .withVelocityY(MaxSpeed.times(lx.getAsDouble()))
                        .withRotationalRate(Constants.MaxAngularRate.times(rx.getAsDouble())))
            .schedule();
        break;
      case ALIGNREEF:
        drivetrain
            .applyRequest(
                () ->
                    drive
                        .withVelocityX(MaxSpeed.times(translation.calculate(drivetrain.getPose().getX(), State.getReefPosition().getPose(State.getTagForTarget(State.getReefPosition())).getX())))
                        .withVelocityY(MaxSpeed.times(translation.calculate(drivetrain.getPose().getY(), State.getReefPosition().getPose(State.getTagForTarget(State.getReefPosition())).getY())))
                        .withRotationalRate(Constants.MaxAngularRate.times(translation.calculate(drivetrain.getPose().getRotation().minus(State.getReefPosition().getPose(State.getTagForTarget(State.getReefPosition())).getRotation().toRotation2d()).getDegrees(), 0))))
            .schedule();
        break;
    }
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
