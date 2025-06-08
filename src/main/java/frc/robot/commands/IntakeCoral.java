// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.arm.Arm;
// import frc.robot.subsystems.arm.Arm.ArmPosition;
// import frc.robot.subsystems.intake.Intake;
// import frc.robot.subsystems.intake.Intake.IntakePosition;
// import org.littletonrobotics.junction.Logger;

// /* You should consider using the more terse Command factories API instead
// https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class IntakeCoral extends Command {
//   Arm arm;
//   Intake intake;
//   boolean hasCoral;

//   /** Creates a new Intake. */
//   public IntakeCoral(Arm arm, Intake intake) {
//     this.arm = arm;
//     this.intake = intake;
//     addRequirements(arm, intake);
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     hasCoral = false;
//     arm.PREPINTAKE().schedule();
//     intake.STOW().schedule();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     if (arm.isAtTarget()
//         && arm.getMode() == ArmPosition.PREPINTAKE
//         && intake.isAtTarget()
//         && intake.getMode() == IntakePosition.STOW
//         && !hasCoral) {
//       intake.INTAKE().schedule();
//       Logger.recordOutput("Intake", 1);
//     } else if (arm.isAtTarget()
//         && intake.isAtTarget()
//         && arm.getMode() == ArmPosition.PREPINTAKE
//         && intake.getMode() == IntakePosition.INTAKE
//         && !hasCoral) {
//       intake.HANDOFF().schedule();
//       Logger.recordOutput("Intake", 2);
//     } else if (arm.isAtTarget()
//         && intake.isAtTarget()
//         && arm.getMode() == ArmPosition.PREPINTAKE
//         && intake.getMode() == IntakePosition.HANDOFF
//         && !hasCoral) {
//       arm.INTAKE().schedule();
//       hasCoral = true;
//       Logger.recordOutput("Intake", 3);
//     } else if (arm.isAtTarget()
//         && intake.isAtTarget()
//         && arm.getMode() == ArmPosition.INTAKE
//         && intake.getMode() == IntakePosition.HANDOFF
//         && hasCoral) {
//       arm.PREPINTAKE().schedule();
//       Logger.recordOutput("Intake", 4);
//     } else if (arm.isAtTarget()
//         && intake.isAtTarget()
//         && arm.getMode() == ArmPosition.PREPINTAKE
//         && intake.getMode() == IntakePosition.HANDOFF
//         && hasCoral) {
//       intake.STOW().schedule();
//       Logger.recordOutput("Intake", 5);
//     }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     arm.CORALIDLE().schedule();
//     Logger.recordOutput("Intake", 6);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return (arm.isAtTarget()
//         && intake.isAtTarget()
//         && arm.getMode() == ArmPosition.PREPINTAKE
//         && intake.getMode() == IntakePosition.STOW
//         && hasCoral);
//   }
// }
