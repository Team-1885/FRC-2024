package frc.robot.commands;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;

public class TurnToHeading extends Command {
    private final DifferentialDrive mDrive;
    private final ADXRS450_Gyro mGyro;
    private final double mTargetHeading;
    private final double P;

    public TurnToHeading(DifferentialDrive pDrive, ADXRS450_Gyro pGyro, double pTargetHeading, double pP) {
        this.mDrive = pDrive;
        this.mGyro = pGyro;
        this.mTargetHeading = pTargetHeading;
        this.P = pP;

        //addRequirements(pDrive);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double mError = mTargetHeading - mGyro.getAngle();
        double mCorrection = P * mError;
        mDrive.tankDrive(mCorrection, -mCorrection);
    }

    @Override
    public boolean isFinished() {
        double mError = mTargetHeading - mGyro.getAngle();
        return Math.abs(mError) < 2;
    }

    @Override
    public void end(boolean interrupted) {
        mDrive.stopMotor();
    }
}
