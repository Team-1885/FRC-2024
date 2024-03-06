 package frc.robot.subsystems.drivetrain;
import org.littletonrobotics.junction.AutoLog;

public interface CANDrivetrainIO {
    @AutoLog
    public static class DriveIOInputs {
        public double leftPositionRad = 0.0;
        public double leftVelocityRadPerSec = 0.0;
        public double leftAppliedVolts = 0.0;
        public double[] leftCurrentAmps = new double[] {};

        public double rightPositionRad = 0.0;
        public double rightVelocityRadPerSec = 0.0;
        public double rightAppliedVolts = 0.0;
        public double[] rightCurrentAmps = new double[] {};
        //public Rotation2d gyroYaw = new Rotation2d();
    }
}