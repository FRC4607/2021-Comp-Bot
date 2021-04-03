package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.lib.drivers.SparkMax;

public class ClimberSubsystem extends SubsystemBase {
    private final CANSparkMax mClimber;
    private final DoubleSolenoid mShifter;

    public boolean mLocked;

    public ClimberSubsystem(CANSparkMax climber, DoubleSolenoid shifter) {
        mClimber = climber;
        mShifter = shifter;

        mLocked = true;
        mShifter.set(Value.kForward);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public void lock() {
        if (!mLocked) {
            mShifter.set(Value.kForward);
            mLocked = true;
        }
    }

    public void unlock() {
        if (mLocked) {
            mShifter.set(Value.kReverse);
            mLocked = false;
        }
    }

    public void toggle() {
        if (mLocked) {
            unlock();
        }
        else {
            lock();
        }
    }

    public void set(double speed) {
        mClimber.set(speed);
    }


    public static ClimberSubsystem create() {
        return new ClimberSubsystem(
                SparkMax.CreateSparkMax(new CANSparkMax(Constants.CLIMBER.MASTER_ID, MotorType.kBrushless)),
                new DoubleSolenoid(Constants.CLIMBER.LOCKED_SOLENOID_ID, Constants.CLIMBER.UNLOCKED_SOLENOID_ID));
    }
}