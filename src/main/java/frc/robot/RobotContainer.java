/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;

import frc.robot.commands.drive.Drive;

import frc.robot.commands.intake.AutoIntake;
import frc.robot.commands.intake.ReverseIntakeOuter;
import frc.robot.commands.intake.ReverseMagazine;
import frc.robot.commands.intake.ReverseThroat;
import frc.robot.commands.intake.RunIntakeOuter;
import frc.robot.commands.intake.RunMagazine;
import frc.robot.commands.intake.RunThroat;
import frc.robot.commands.intake.StopIntakeOuter;
import frc.robot.commands.intake.StopMagazine;
import frc.robot.commands.intake.StopThroat;

import frc.robot.commands.shooter.AutoShooter;
import frc.robot.commands.shooter.RunShooter;
import frc.robot.commands.shooter.StopShooter;
import frc.robot.commands.shooter.ToggleShooter;
import frc.robot.commands.shooter.RunFeeder;
import frc.robot.commands.shooter.ReverseFeeder;
import frc.robot.commands.shooter.StopFeeder;

import frc.robot.triggers.DPadDown;
import frc.robot.triggers.DPadDownLeft;
import frc.robot.triggers.DPadDownRight;
import frc.robot.triggers.DPadLeft;
import frc.robot.triggers.DPadRight;
import frc.robot.triggers.DPadUp;
import frc.robot.triggers.DPadUpLeft;
import frc.robot.triggers.DPadUpRight;
import frc.robot.triggers.LimitSwitch;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TrenchSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final DriveSubsystem m_drive = new DriveSubsystem();
    private final IntakeSubsystem m_intake = new IntakeSubsystem();
    private final ShooterSubsystem m_shooter = new ShooterSubsystem();
    // private final TrenchSubsystem m_trench = new TrenchSubsystem();

    // The default driving command
    public final Drive m_driveCommand = new Drive(m_drive);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        // set the default drive command to split-stick tank drive
        m_drive.setDefaultCommand(m_driveCommand);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        new JoystickButton(OIConstants.joysticks[OIConstants.kAutoIntakeControl[0]], OIConstants.kAutoIntakeControl[1])
                .whenHeld(new AutoIntake(m_intake, new LimitSwitch(IntakeConstants.kBaseSwitch),
                        new LimitSwitch(IntakeConstants.kMagSwitch), new LimitSwitch(IntakeConstants.kTopSwitch)));

        new JoystickButton(OIConstants.joysticks[OIConstants.kAutoShooterControl[0]],
                OIConstants.kAutoShooterControl[1]).whenHeld(new AutoShooter(m_shooter, m_drive));

        new JoystickButton(OIConstants.joysticks[OIConstants.kCurvatureDriveQuickTurnToggle[0]],
                OIConstants.kCurvatureDriveQuickTurnToggle[1]).whenPressed(() -> m_drive.setCurvatureQuickTurn(true))
                        .whenReleased(() -> m_drive.setCurvatureQuickTurn(false));

        new DPadUp(OIConstants.kShooterFeederStick).or(new DPadUpLeft(OIConstants.kShooterFeederStick))
                .or(new DPadUpRight(OIConstants.kShooterFeederStick)).whenActive(() -> new RunShooter(m_shooter))
                .whenInactive(() -> new StopShooter(m_shooter));

        new DPadRight(OIConstants.kShooterFeederStick).or(new DPadUpRight(OIConstants.kShooterFeederStick))
                .whenActive(() -> new RunFeeder(m_shooter)).whenInactive(() -> new StopFeeder(m_shooter));

        new DPadLeft(OIConstants.kShooterFeederStick).or(new DPadUpLeft(OIConstants.kShooterFeederStick))
                .whenActive(() -> new ReverseFeeder(m_shooter)).whenInactive(() -> new StopFeeder(m_shooter));

        new DPadDown(OIConstants.kShooterFeederStick).whenActive(() -> new ToggleShooter(m_shooter));

        new DPadUp(OIConstants.kIntakeStick).or(new DPadUpLeft(OIConstants.kIntakeStick))
                .or(new DPadUpRight(OIConstants.kIntakeStick)).whenActive(new RunIntakeOuter(m_intake))
                .whenActive(new RunThroat(m_intake)).whenInactive(new StopIntakeOuter(m_intake))
                .whenInactive(new StopThroat(m_intake));

        new DPadRight(OIConstants.kIntakeStick).or(new DPadUpRight(OIConstants.kIntakeStick))
                .or(new DPadDownRight(OIConstants.kIntakeStick)).whenActive(new RunMagazine(m_intake))
                .whenInactive(new StopMagazine(m_intake));

        new DPadDown(OIConstants.kIntakeStick).or(new DPadDownLeft(OIConstants.kIntakeStick))
                .or(new DPadDownRight(OIConstants.kIntakeStick)).whenActive(new ReverseIntakeOuter(m_intake))
                .whenActive(new ReverseThroat(m_intake)).whenInactive(new StopIntakeOuter(m_intake))
                .whenInactive(new StopThroat(m_intake));

        new DPadLeft(OIConstants.kIntakeStick).or(new DPadUpLeft(OIConstants.kIntakeStick))
                .or(new DPadDownLeft(OIConstants.kIntakeStick)).whenActive(new ReverseMagazine(m_intake))
                .whenInactive(new StopMagazine(m_intake));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return null;
    }
}
