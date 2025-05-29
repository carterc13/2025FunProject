package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.bot.Bot;
import frc.robot.subsystems.bot.Bot.BotDificulty;
import frc.robot.subsystems.bot.Bot.BotType;
import frc.robot.utils.TunableController;
import frc.robot.utils.TunableController.TunableControllerType;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
    private final boolean testingBot = true;    

    private final BotType botType = BotType.CYCLES;
    private final BotDificulty botDificulty = BotDificulty.FRIGHTENED;

    private final Bot bot;
    // private final RealDriver driver;

    public RobotContainer() {
        bot = new Bot(testingBot, botType, botDificulty);
        // driver = new RealDriver(testingBot);

        

        configureBindings();
    }

    private void configureBindings() {}

    public Command getBotAutonomousCommand() {
        return bot.getAutonomousCommand();
    }
    public void dissableBotTemp() {
        bot.dissableBotTemp();
    }
    public void undissableBotTemp() {
        bot.undissableBotTemp();
    }
}
