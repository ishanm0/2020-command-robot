/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

/**
 * Intake Subsystem, contains objects and methods needed to use and control the intake
 */
public class IntakeSubsystem extends SubsystemBase {
    private final DoubleSolenoid m_intakeSolenoid = new DoubleSolenoid(IntakeConstants.kIntakeSolenoidPorts[0], IntakeConstants.kIntakeSolenoidPorts[1]);
    
    private final WPI_TalonSRX m_intakeOuter = new WPI_TalonSRX(IntakeConstants.kIntakeTalonOuterPort);
    
    private final WPI_TalonSRX m_intakeLeft = new WPI_TalonSRX(IntakeConstants.kIntakeTalonFrontPort);
    private final WPI_TalonSRX m_intakeRight = new WPI_TalonSRX(IntakeConstants.kIntakeTalonBackPort);
    
    private final WPI_TalonSRX m_magazineLeft = new WPI_TalonSRX(IntakeConstants.kMagazineTalonLeftPort);
    private final WPI_TalonSRX m_magazineRight = new WPI_TalonSRX(IntakeConstants.kMagazineTalonRightPort);

    /**
     * Creates a new IntakeSubsystem, inverts necessary talons.
     */
    public IntakeSubsystem() {
        m_intakeOuter.setInverted(false);

        m_intakeLeft.setInverted(false);
        m_intakeRight.setInverted(true);

        m_magazineLeft.setInverted(true);
        m_magazineRight.setInverted(false);
    }

    /**
     * Lowers the intake arm.
     */
    public void lowerIntake() {
        m_intakeSolenoid.set(Value.kForward);
    }

    /**
     * Raises the intake arm.
     */
    public void raiseIntake() {
        m_intakeSolenoid.set(Value.kReverse);
    }

    /**
     * Runs the throat wheels at set speeds.
     */
    public void runThroat() {
        m_intakeLeft.set(IntakeConstants.kThroatSpeed);
        m_intakeRight.set(IntakeConstants.kThroatSpeed);
    }

    /**
     * Runs the outer intake wheels at set speeds.
     */
    public void runIntakeOuter() {
        m_intakeOuter.set(IntakeConstants.kIntakeOuterSpeed);
    }
    
    /**
     * Runs the magazine wheels at set speeds.
     */
    public void runMagazine() {
        m_magazineLeft.set(IntakeConstants.kMagazineSpeed);
        m_magazineRight.set(IntakeConstants.kMagazineSpeed);
    }

    /**
     * Runs the throat wheels at set speeds in reverse.
     */
    public void reverseThroat() {
        m_intakeLeft.set(-IntakeConstants.kThroatSpeed);
        m_intakeRight.set(-IntakeConstants.kThroatSpeed);
    }

    /**
     * Runs the outer intake wheels at set speeds in reverse.
     */
    public void reverseIntakeOuter() {
        m_intakeOuter.set(-IntakeConstants.kIntakeOuterSpeed);
    }
    
    /**
     * Runs the magazine wheels at set speeds in reverse.
     */
    public void reverseMagazine() {
        m_magazineLeft.set(-IntakeConstants.kMagazineSpeed);
        m_magazineRight.set(-IntakeConstants.kMagazineSpeed);
    }

    /**
     * Stops the throat wheels.
     */
    public void stopThroat() {
        m_intakeLeft.set(0);
        m_intakeRight.set(0);
    }

    /**
     * Stops the outer intake wheels
     */
    public void stopIntakeOuter() {
        m_intakeOuter.set(0);
    }
    
    /**
     * Stops the magazine wheels.
     */
    public void stopMagazine() {
        m_magazineLeft.set(0);
        m_magazineRight.set(0);
    }
}
