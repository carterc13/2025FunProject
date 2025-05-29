package frc.robot.subsystems.bot.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ElevatorIOSIM extends ElevatorIOCTRE {
  private final DCMotor motor = DCMotor.getKrakenX60(4);
  private final PWMTalonFX talonfx = new PWMTalonFX(2);
  private final PIDController controller = new PIDController(7.0, 0.0, 0.25);
  private double setpoint = 0;

  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          motor,
          3,
          Units.lbsToKilograms(30),
          Units.inchesToMeters(2),
          0,
          Units.inchesToMeters(80),
          true,
          0);

  private final LoggedMechanismLigament2d m_arm;

  public ElevatorIOSIM(LoggedMechanismRoot2d m_armPivot) {
    m_arm =
        m_armPivot.append(
            new LoggedMechanismLigament2d("Elevator", 0, 0, 5, new Color8Bit(Color.kGreen)));
    SmartDashboard.putNumber("EleP", controller.getP());
    SmartDashboard.putNumber("EleI", controller.getI());
    SmartDashboard.putNumber("EleD", controller.getD());

    SmartDashboard.putNumber("Coral/x", 0);
    SmartDashboard.putNumber("Coral/y", 0);
    SmartDashboard.putNumber("Coral/z", 0);
    SmartDashboard.putNumber("Coral/roll", 0);
    SmartDashboard.putNumber("Coral/pitch", 0);
    SmartDashboard.putNumber("Coral/yaw", 0);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    controller.setP(SmartDashboard.getNumber("EleP", 0));
    controller.setI(SmartDashboard.getNumber("EleI", 0));
    controller.setD(SmartDashboard.getNumber("EleD", 0));

    talonfx.setVoltage(
        controller.calculate((m_elevatorSim.getPositionMeters()), Units.inchesToMeters(setpoint)));

    m_elevatorSim.setInput(talonfx.get() * RobotController.getBatteryVoltage());

    m_elevatorSim.update(0.020);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));

    m_arm.setLength(m_elevatorSim.getPositionMeters() + Units.inchesToMeters(27));

    m_arm.setAngle(SmartDashboard.getNumber("Arm Angle", 0));

    inputs.distance = Meters.of(m_elevatorSim.getPositionMeters());

    inputs.setpoint = Inches.of(setpoint);
  }

  @Override
  public void setDistance(Distance distance) {
    setpoint = distance.in(Inches);
  }
}
