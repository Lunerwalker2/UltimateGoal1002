package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.Util.HardwareNames;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class DriveBase extends Component {


    public DcMotorEx left_front_drive;
    public DcMotorEx left_back_drive;
    public DcMotorEx right_front_drive;
    public DcMotorEx right_back_drive;

    List<DcMotorEx> motors;

    public DriveBase(Robot robot){
        super(robot);


        left_front_drive = findMotor(HardwareNames.Drive.LEFT_FRONT);
        left_back_drive = findMotor(HardwareNames.Drive.LEFT_BACK);
        right_front_drive = findMotor(HardwareNames.Drive.RIGHT_FRONT);
        right_back_drive = findMotor(HardwareNames.Drive.RIGHT_BACK);

        right_front_drive.setDirection(DcMotorEx.Direction.REVERSE);
        right_back_drive.setDirection(DcMotorEx.Direction.REVERSE);

        motors = Arrays.asList(left_front_drive, left_back_drive, right_front_drive, right_back_drive);

        motors.forEach((DcMotorEx motor) -> motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE));



        motors.forEach((motor) -> motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER));

        setMaxRPMFraction(1.0);

    }

    private DcMotorEx findMotor(String name){
        return hardwareMap.get(DcMotorEx.class, name);
    }

    /**
     * Update the drive base using the
     * {@link DriveFields#movement_x}, {@link DriveFields#movement_y}, and
     * {@link DriveFields#movement_turn}. This uses those components and turns them into
     * drive powers with the order of {@code {lf, lb, rf, rb}}. Then it updates the drive motors.
     */
    public void updateHolonomic(){

        Pose2d vel = DriveFields.normalizedVels();

        List<Double> drivePowers = MecanumKinematics.robotToWheelVelocities(vel, 1.0, 1.0, DriveFields.lateralMultiplier);

        DriveFields.distributePowers(drivePowers);

        setPower();
    }

    /**
     * Simply updates motor powers with stored powers
     */
    public void updatePowers(){
        setPower();
    }


    /**
     * Set PIDF coefficients to all drive motors
     * @param coefficients The PIDF coefficients
     * @param runMode The mode to set them as (usually RUN_USING_ENCODER)
     */
    public void setPIDFCoefficients(PIDFCoefficients coefficients, DcMotorEx.RunMode runMode){
        motors.forEach((motor) -> motor.setPIDFCoefficients(runMode, coefficients));
    }

    public PIDFCoefficients getPIDCoefficients(DcMotorEx.RunMode runMode){
        return left_front_drive.getPIDFCoefficients(runMode);//Random motor :)
    }

    public void setMaxRPMFraction(double fraction){
        motors.forEach((motor) -> {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(fraction);
            motor.setMotorType(motorConfigurationType);
        });
    }

    //Send the powers to the motors
    public void setPower(){
        left_front_drive.setPower(DriveFields.lf_power);
        left_back_drive.setPower(DriveFields.lb_power);
        right_front_drive.setPower(DriveFields.rf_power);
        right_back_drive.setPower(DriveFields.rb_power);
    }

    public void stop(){
        DriveFields.distributePowers(new double[] {0,0,0,0});
        setPower();
    }
    //RR uses a different order than us, so the following methods are specific to that


    public List<Double> getWheelEncoderPositionsRR(){
        return Arrays.asList(
                (double) left_front_drive.getCurrentPosition(),
                (double) left_back_drive.getCurrentPosition(),
                (double) right_back_drive.getCurrentPosition(),
                (double) right_front_drive.getCurrentPosition()

        );
    }

    public List<Double> getWheelEncoderVelocitiesRR(){
        return Arrays.asList(
                left_front_drive.getVelocity(),
                left_back_drive.getVelocity(),
                right_front_drive.getVelocity(),
                right_back_drive.getVelocity()
        );
    }




}
