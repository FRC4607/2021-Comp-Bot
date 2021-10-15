package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShifterSubsystem;
import frc.robot.subsystems.TransferWheelSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class Auton_SixBallReducedSpeed extends CommandBase {
    private final DrivetrainSubsystem mDrivetrain;
    private final ShifterSubsystem mShifter;
    private final TurretSubsystem mTurret;
    private final FlywheelSubsystem mFlywheel;
    private final CommandScheduler mScheduler;

    private final TransferWheelSubsystem mTransferWheel;
    private final HopperSubsystem mHopperMotor;
    private final IndexerSubsystem mIndexerMotor;

    private final IntakeSubsystem mIntake;

    public Auton_SixBallReducedSpeed(DrivetrainSubsystem drivetrain, ShifterSubsystem shifter, TurretSubsystem turret, FlywheelSubsystem flywheel,
            TransferWheelSubsystem wheel, HopperSubsystem hopper, IndexerSubsystem index, IntakeSubsystem intake) {
        mDrivetrain = drivetrain;
        mShifter = shifter;
        mTurret = turret;
        mFlywheel = flywheel;
        mScheduler = CommandScheduler.getInstance();
        mTransferWheel = wheel;
        mHopperMotor = hopper;
        mIndexerMotor = index;
        mIntake = intake;
        addRequirements(mDrivetrain, mShifter, mTurret, mTransferWheel, mHopperMotor, mIndexerMotor);
    }

    @Override
    public void initialize() {
        mIntake.extend();
        mScheduler.schedule(new TurretAlignment(mTurret).withTimeout(0.2));
        mScheduler.schedule(new TurretAlignment(mTurret).withTimeout(1).andThen(() -> {
            mFlywheel.setClosedLoop(5000);
            try {
                Thread.sleep(2000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            mScheduler.schedule(new AutonFlywheel(mTransferWheel, mHopperMotor, mIndexerMotor).withTimeout(2).andThen(() -> {
                mScheduler.schedule(new ParallelDeadlineGroup(new DriveWithGyroGears(mDrivetrain, mShifter, 4.5, 0, 0.9, false).andThen(() -> {
                    mIntake.setRoller(0);
                    mScheduler.schedule(new DriveWithGyroGears(mDrivetrain, mShifter, 2.5, 0, -0.65, true).andThen(() -> {
                        mScheduler.schedule(new TurretAlignment(mTurret).withTimeout(1).andThen(() -> {
                            mScheduler.schedule(new AutonFlywheel(mTransferWheel, mHopperMotor, mIndexerMotor).withTimeout(999).andThen(() -> {
                                mFlywheel.setOpenLoop();
                            }, mFlywheel, mDrivetrain));
                        }, mFlywheel, mDrivetrain));
                    }, mFlywheel, mDrivetrain));
                }, mFlywheel, mDrivetrain), new AutonIntake(mIntake)));
            }, mFlywheel, mDrivetrain));
        }, mFlywheel, mDrivetrain));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
