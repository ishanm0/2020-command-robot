package frc.robot.commands.trench;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.TrenchConstants;
import frc.robot.subsystems.TrenchSubsystem;

/**
 * Control Panel/Trench Rotation control (PID only so far, PID on RoboRIO)
 */
public class Rotation2 extends PIDCommand {
    public Rotation2(TrenchSubsystem m_trench) {
        super(new PIDController(TrenchConstants.kP, TrenchConstants.kI, TrenchConstants.kD),
                m_trench::getTrenchTalonRate, TrenchConstants.kRPM, output -> m_trench.getTrenchTalon().set(output),
                m_trench);
    }
}