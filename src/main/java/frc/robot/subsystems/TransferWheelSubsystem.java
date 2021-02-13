package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.lib.drivers.SparkMax;

public class TransferWheelSubsystem extends SubsystemBase {
    private final CANSparkMax mTransferWheel;

    public boolean mBackwards;

    public TransferWheelSubsystem(CANSparkMax transferWheel) {
        mTransferWheel = transferWheel;
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
        mTransferWheel.set(0.0);
    }

    public void enable() {
        if (mBackwards) {
            mTransferWheel.set(-Constants.TRANSFER_WHEEL.SPEED);
        }
        else {
            mTransferWheel.set(Constants.TRANSFER_WHEEL.SPEED);
        }
    }

    public static TransferWheelSubsystem create() {
        return new TransferWheelSubsystem(SparkMax.CreateSparkMax(new CANSparkMax(Constants.TRANSFER_WHEEL.MASTER_ID, MotorType.kBrushless)));
    }
}
