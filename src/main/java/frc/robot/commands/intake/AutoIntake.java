package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.StartEndCommand;

import frc.robot.subsystems.IntakeSubsystem;

import frc.robot.triggers.LimitSwitch;
import frc.robot.triggers.ToggleButton;

public class AutoIntake extends StartEndCommand {
    private final IntakeSubsystem m_intake;
    private final LimitSwitch m_baseSwitch;
    private final LimitSwitch m_magSwitch;
    private final LimitSwitch m_topSwitch;

    private boolean magOn = false;
    private boolean magIsOn = false;

    private boolean feederStorage = true;

    private ToggleButton feederStorageButton = new ToggleButton("feederStorage", feederStorage);

    /**
     * Creates a new AutoIntake.
     * 
     * @param intake The intake subsystem this command will run on.
     */
    public AutoIntake(IntakeSubsystem intake, LimitSwitch base, LimitSwitch mag, LimitSwitch top) {
        super(() -> new StartIntake(intake), () -> new FinishIntake(intake));
        m_intake = intake;
        addRequirements(m_intake);

        m_baseSwitch = base;
        m_magSwitch = mag;
        m_topSwitch = top;
    }

    @Override
    public void execute() {
        feederStorage = feederStorageButton.get();

        if (!m_baseSwitch.get() || (feederStorage && m_topSwitch.get()) || (!feederStorage && m_magSwitch.get())) {
            magOn = false;
        } else if (m_baseSwitch.get()) {
            magOn = true;
        }

        if (magOn && !magIsOn) {
            new RunMagazine(m_intake);
            magIsOn = true;
        } else if (!magOn && magIsOn) {
            new StopMagazine(m_intake);
            magIsOn = false;
        }
    }

    @Override
    public boolean isFinished() {
        return m_baseSwitch.get() && ((feederStorage && m_topSwitch.get()) || (!feederStorage && m_magSwitch.get()));
    }
}