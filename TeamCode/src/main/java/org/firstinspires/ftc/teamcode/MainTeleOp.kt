package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics.robotToWheelVelocities
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import kotlin.math.abs


@TeleOp(name = "WoodBotTeleOp")
class MainTeleOp : LinearOpMode() {

    companion object {
        val VX_WEIGHT = 1.0
        val VY_WEIGHT = 1.0
        val OMEGA_WEIGHT = 1.0
    }

    lateinit var lf: DcMotorEx
    lateinit var lb: DcMotorEx
    lateinit var rf: DcMotorEx
    lateinit var rb: DcMotorEx


    var slowModeMult = 0.7


    val motors = ArrayList<DcMotorEx>()



    @Throws(InterruptedException::class)
    override fun runOpMode() {

        lf = hardwareMap.get(DcMotorEx::class.java, "lf")
        lb = hardwareMap.get(DcMotorEx::class.java, "lb")
        rf = hardwareMap.get(DcMotorEx::class.java, "rf")
        rb = hardwareMap.get(DcMotorEx::class.java, "rb")


        motors.add(lf);
        motors.add(lb);
        motors.add(rf);
        motors.add(rb);

        motors.forEach { motor: DcMotorEx -> motor.mode = DcMotor.RunMode.RUN_USING_ENCODER }
        motors.forEach { motor: DcMotorEx -> motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE }

        motors.forEach { motor: DcMotorEx ->
            val motorConfigurationType = motor.motorType.clone()
            motorConfigurationType.achieveableMaxRPMFraction = 1.0
            motor.motorType = motorConfigurationType
        }

        rf.direction = DcMotorSimple.Direction.REVERSE
        rb.direction = DcMotorSimple.Direction.REVERSE


        waitForStart()

        while (opModeIsActive()){
            if(isStopRequested) return


            val roadRunnerVoodooPowers = Pose2d((-gamepad1.left_stick_y).toDouble(), -gamepad1.left_stick_x.toDouble(), gamepad1.right_stick_x.toDouble())
            val drivePowers = getDrivePowers(normalizedVels(roadRunnerVoodooPowers))


            slowModeMult = if (gamepad1.left_bumper) 0.5 else 1.0



        }

    }
    fun getDrivePowers(drivePower: Pose2d): List<Double> {
        return robotToWheelVelocities(drivePower,
                1.0, 1.0, 1.0)
    }

    fun normalizedVels(drivePower: Pose2d): Pose2d {
        var vel = drivePower
        if ((abs(drivePower.x) + abs(drivePower.y)
                        + abs(drivePower.heading)) > 1) {
            // re-normalize the powers according to the weights
            val denom = VX_WEIGHT * abs(drivePower.x) + SampleMecanumDrive.VY_WEIGHT * abs(drivePower.y) + OMEGA_WEIGHT * abs(drivePower.heading)
            vel = Pose2d(
                    VX_WEIGHT * drivePower.x,
                    SampleMecanumDrive.VY_WEIGHT * drivePower.y,
                    OMEGA_WEIGHT * drivePower.heading
            ).div(denom)
        }
        return vel
    }
}