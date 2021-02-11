package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final DoubleSolenoid mArms;
    private final WPI_TalonSRX mRoller;

    private boolean mExtended = false;

    public IntakeSubsystem(DoubleSolenoid shifter, WPI_TalonSRX roller) {
        mArms = shifter;
        mRoller = roller;
        mExtended = false;

        mArms.set(kReverse);

        mRoller.configContinuousCurrentLimit( Constants.CURRENT_LIMIT.TALON_AMPS_LIMIT );
        mRoller.configPeakCurrentLimit( Constants.CURRENT_LIMIT.TALON_AMPS_LIMIT );
        mRoller.configPeakCurrentDuration( Constants.GLOBAL.TALON_CURRENT_LIMIT_TIMEOUT_MS );
        mRoller.enableCurrentLimit( true );
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    private void extend() {
        mArms.set(kForward);
        mExtended = true;
    }

    private void retract() {
        mArms.set(kReverse);
        mExtended = false;
    }
    
    public void toggleIntake() {
        if (mExtended) {
            retract();
        }
        else {
            extend();
        }
    }

    public void setRoller(double speed) {
        if (Math.abs(speed) < Constants.INTAKE.DEADBAND) {
            mRoller.set( 0.0 );
        }
        else {
            mRoller.set(-speed * 0.5);
        }
    }

    public static IntakeSubsystem create() {
        return new IntakeSubsystem(new DoubleSolenoid(Constants.INTAKE.UP_SOLENOID_ID, Constants.INTAKE.DOWN_SOLENOID_ID), new WPI_TalonSRX(Constants.INTAKE.MASTER_ID));
    }
}

