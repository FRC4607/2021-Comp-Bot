package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.lib.drivers.SparkMax;

public class HopperSubsystem extends SubsystemBase {
    private final CANSparkMax mHopper;

    public boolean mBackwards;

    public HopperSubsystem(CANSparkMax hopper) {
        mHopper = hopper;
        mBackwards = false;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public void disable() {
        mHopper.set(0.0);
    }

    public void enable() {
        if (mBackwards) {
            mHopper.set(-Constants.HOPPER.SPEED);
        }
        else {
            mHopper.set(Constants.HOPPER.SPEED);
        }
    }

    public static HopperSubsystem create() {
        return new HopperSubsystem(SparkMax.CreateSparkMax(new CANSparkMax(Constants.HOPPER.MASTER_ID, MotorType.kBrushless)));
    }
}