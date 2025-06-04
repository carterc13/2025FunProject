// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import static edu.wpi.first.units.Units.Degrees;

// import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
// import com.ctre.phoenix6.swerve.SwerveRequest;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.units.measure.LinearVelocity;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.generated.TunerConstants;
// import frc.robot.subsystems.drive.Drive;
// import frc.robot.utils.PoseComputer;
// import java.util.function.DoubleSupplier;
// import org.littletonrobotics.junction.Logger;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class DriveHandler extends Command {
//   LinearVelocity MaxSpeed = TunerConstants.kSpeedAt12Volts;
//   SwerveRequest.FieldCentric drive =
//       new SwerveRequest.FieldCentric()
//           .withDeadband(MaxSpeed.times(0.1))
//           .withRotationalDeadband(Constants.MaxAngularRate.times(0.1)) // Add a 10% deadband
//           .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
//   Drive drivetrain;
//   DoubleSupplier ly;
//   DoubleSupplier lx;
//   DoubleSupplier rx;
//   PIDController translation = new PIDController(1, 0, 0.001);
//   PIDController rotation = new PIDController(0.04, 0, 0.00075);

//   public DriveHandler(Drive drivetrain, DoubleSupplier ly, DoubleSupplier lx, DoubleSupplier rx) {
//     this.drivetrain = drivetrain;
//     this.ly = ly;
//     this.lx = lx;
//     this.rx = rx;
//     addRequirements(drivetrain);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     drivetrain.setRightSource(PoseComputer.isRightSource(() -> drivetrain.getPose().getY()));
//     Logger.recordOutput("omgwhyisnthtisliterallyworking", drivetrain.getDriveState());
//     switch (drivetrain.getDriveState()) {
//       case IDLE:
//         drivetrain
//             .applyRequest(
//                 () ->
//                     drive
//                         .withVelocityX(MaxSpeed.times(ly.getAsDouble()))
//                         .withVelocityY(MaxSpeed.times(lx.getAsDouble()))
//                         .withRotationalRate(Constants.MaxAngularRate.times(rx.getAsDouble())))
//             .schedule();
//         break;
//       case ALIGNREEF:
//         drivetrain
//             .applyRequest(
//                 () ->
//                     drive
//                         .withVelocityX(
//                             MaxSpeed.times(
//                                 -translation.calculate(
//                                     drivetrain.getPose().getX(),
//                                     drivetrain.getReefPosition().getPose().getX())))
//                         .withVelocityY(
//                             MaxSpeed.times(
//                                 -translation.calculate(
//                                     drivetrain.getPose().getY(),
//                                     drivetrain.getReefPosition().getPose().getY())))
//                         .withRotationalRate(
//                             Constants.MaxAngularRate.times(
//                                 rotation.calculate(
//                                     drivetrain
//                                         .getPose()
//                                         .getRotation()
//                                         .minus(
//                                             drivetrain
//                                                 .getReefPosition()
//                                                 .getPose()
//                                                 .getRotation()
//                                                 .toRotation2d())
//                                         .minus(new Rotation2d(Degrees.of(180)))
//                                         .getDegrees(),
//                                     0))))
//             .schedule();
//         break;
//       default:
//         break;
//     }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
