
package org.firstinspires.ftc.teamcode.TeleOp;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "WoodBotTeleOp", group = "Jank")
public class JankWoodTeleOp extends LinearOpMode {


    static final double VX_WEIGHT = 1;
    static final double VY_WEIGHT = 1;
    static final double OMEGA_WEIGHT = 1;

    static final double goalClawOpen = 0.8;
    static final double goalClawClose = 0.4;

    static final double ringClawOpen = 0.0;
    static final double ringClawClose = 0.6;

    DcMotorEx lf;
    DcMotorEx lb;
    DcMotorEx rf;
    DcMotorEx rb;

    DcMotorEx goalArm;
    DcMotorEx ringArm;

    Servo goalClaw;
    Servo ringClaw;

    private double slowModeMult = 0.7;


    private boolean ringClawBounce = false;
    private boolean ringClawChanged = false;

    private boolean goalClawChanged = false;
    private boolean goalClawBounce = false;

    ArrayList<DcMotorEx> motors = new ArrayList<>();


    @Override
    public void runOpMode() throws InterruptedException {


        lf = hardwareMap.get(DcMotorEx.class, "lf");
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rb = hardwareMap.get(DcMotorEx.class, "rb");

        goalArm = hardwareMap.get(DcMotorEx.class, "goal arm");
        ringArm = hardwareMap.get(DcMotorEx.class, "ring arm");

        goalClaw = hardwareMap.get(Servo.class, "goal claw");
        ringClaw = hardwareMap.get(Servo.class, "ring claw");

        motors.add(lf);
        motors.add(lb);
        motors.add(rf);
        motors.add(rb);

        motors.forEach((motor) -> motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER));
        motors.forEach((motor) -> motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE));

        motors.forEach((motor) -> {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        });

        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);


        goalArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        goalArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        goalArm.setDirection(DcMotorSimple.Direction.REVERSE);

        ringArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ringArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();

        goalClaw.setPosition(goalClawOpen);
        ringClaw.setPosition(ringClawOpen);


        while (opModeIsActive()){
            if (isStopRequested()) return;

            if(gamepad2.a && !goalClawBounce){
                moveGoalClaw();
                goalClawBounce = true;
            } else if(!gamepad2.a){
                goalClawBounce = false;
            }


            if(gamepad2.left_bumper && !ringClawBounce){
                moveRingClaw();
                ringClawBounce = true;
            } else if(!gamepad2.left_bumper){
                ringClawBounce = false;
            }


            if(gamepad2.dpad_up) ringArm.setPower(0.8);
            else if(gamepad2.dpad_down) ringArm.setPower(-0.8);
            else ringArm.setPower(0);

            if(gamepad2.left_trigger >= 0.2) goalArm.setPower(0.4);
            else if(gamepad2.right_trigger >= 0.2) goalArm.setPower(-0.2);
            else goalArm.setPower(0);

            //We use RoadRunner's reverse kinematics for wheel powers
            /* It's some weird voodoo magic that somehow works.
            It requires us to reverse the previous gamepad values in ways that are frankly weird.
            But it does math for us so we do it :)
             */

            Pose2d roadRunnerVoodooPowers = new Pose2d(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            List<Double> drivePowers = getDrivePowers(normalizedVels(roadRunnerVoodooPowers));

            slowModeMult = gamepad1.left_bumper ? 0.5 : 1;


            lf.setPower(drivePowers.get(0) * slowModeMult);
            lb.setPower(drivePowers.get(1) * slowModeMult);
            rf.setPower(drivePowers.get(3) * slowModeMult);
            rb.setPower(drivePowers.get(2) * slowModeMult);

        }

        motors.forEach((motor) -> motor.setPower(0));
        goalArm.setPower(0);



    }

    private void moveGoalClaw(){
        if(!goalClawChanged){
            goalClaw.setPosition(goalClawOpen);
            goalClawChanged = true;
        } else {
            goalClaw.setPosition(goalClawClose);
            goalClawChanged = false;
        }
    }

    private void moveRingClaw(){
        if(!ringClawChanged){
            ringClaw.setPosition(ringClawOpen);
            ringClawChanged = true;
        } else {
            ringClaw.setPosition(ringClawClose);
            ringClawChanged = false;
        }
    }

    public static List<Double> getDrivePowers(Pose2d drivePower){
        return MecanumKinematics.robotToWheelVelocities(drivePower,
                1.0, 1.0, 1.0);
    }
    public static Pose2d normalizedVels(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }
        return vel;
    }

}
