package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.lib.drivers.SparkMax;

public class IndexerSubsystem extends SubsystemBase {
    private final CANSparkMax mIndexer;

    public boolean mBackwards;

    public IndexerSubsystem(CANSparkMax indexer) {
        mIndexer = indexer;
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
        mIndexer.set(0.0);
    }

    public void enable() {
        if (mBackwards) {
            mIndexer.set(-Constants.INDEXER.SPEED);
        }
        else {
            mIndexer.set(Constants.INDEXER.SPEED);
        }
    }

    public static IndexerSubsystem create() {
        return new IndexerSubsystem(SparkMax.CreateSparkMax(new CANSparkMax(Constants.INDEXER.MASTER_ID, MotorType.kBrushless)));
    }
}