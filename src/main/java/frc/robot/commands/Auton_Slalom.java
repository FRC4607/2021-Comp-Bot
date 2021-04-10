/*package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShifterSubsystem;

public class Auton_Slalom extends CommandBase {
    private final DrivetrainSubsystem mDrivetrain;
    private final ShifterSubsystem mShifter;

    private final CommandScheduler mScheduler;

    private final double mSpeed = 0.5;

    public Auton_Slalom(DrivetrainSubsystem drivetrain, ShifterSubsystem shifter) {
        mDrivetrain = drivetrain;
        mShifter = shifter;

        mScheduler = CommandScheduler.getInstance();

        addRequirements(mDrivetrain, mShifter);
    }

    private Command driveIn(double in, double heading) {
        return driveM(in * 0.0254, heading);
    }

    private Command driveM(double m, double heading) {
        new DriveWithGyroGears(mDrivetrain, mShifter, m, heading, mSpeed, true);
    }

    private Command circleIn(double in, boolean cw, double degrees) {
        return circleM(in * 0.0254, cw, degrees);
    }

    private Command circleM(double m, boolean cw, double degrees) {
        new DriveInCircle(mDrivetrain, mShifter, m, cw, degrees, mSpeed, true);
    }

    @Override
    public void initialize() {
        // Rotations work like in calculus with quadrants I, II, III, and IV in a ccw order;
        // 60 * sqrt(2) because 45 45 90 triangle
        mScheduler.schedule(driveIn(84.85, 45).andThen(() -> {
            driveIn(120, 0).andThen(() -> {
                //30 * sqrt(2)
                driveIn(42.43, -45).andThen(() -> {
                    circleIn(30, false, 360).andThen(() -> {
                        
                    }, mDrivetrain, mShifter);
                }, mDrivetrain, mShifter);
            }, mDrivetrain, mShifter);
        }, mDrivetrain, mShifter));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}*/