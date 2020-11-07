package org.firstinspires.ftc.teamcode.Auto

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Vision.pipelines.RingStackPipeline
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvInternalCamera2


@Autonomous(name = "WoodBotAuto", group = "Jank")
class WoodBotAuto: LinearOpMode() {


    lateinit var lf: DcMotorEx
    lateinit var lb: DcMotorEx
    lateinit var rf: DcMotorEx
    lateinit var rb: DcMotorEx

    lateinit var imu: BNO055IMU

    lateinit var phoneCam: OpenCvCamera

    private val visionPipeline = RingStackPipeline()

    private val motors = ArrayList<DcMotorEx>()

    private val turnCoefficient = 0.05

    private val forwardPower = 0.7

    private val backwardPower = 0.5

    private val timer = ElapsedTime(ElapsedTime.Resolution.SECONDS)


    @Throws(InterruptedException::class)
    override fun runOpMode(){


        lf = hardwareMap.get(DcMotorEx::class.java, "lf")
        lb = hardwareMap.get(DcMotorEx::class.java, "lb")
        rf = hardwareMap.get(DcMotorEx::class.java, "rf")
        rb = hardwareMap.get(DcMotorEx::class.java, "rb")

        imu = hardwareMap.get(BNO055IMU::class.java, "imu")

        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName)
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId)

        phoneCam.setPipeline(visionPipeline)



        val imuParams = BNO055IMU.Parameters()
        imuParams.angleUnit = BNO055IMU.AngleUnit.DEGREES
        imu.initialize(imuParams)

        motors.add(lf)
        motors.add(lb)
        motors.add(rf)
        motors.add(rb)

        motors.forEach { motor: DcMotorEx -> motor.mode = DcMotor.RunMode.RUN_USING_ENCODER }
        motors.forEach { motor: DcMotorEx -> motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE }

        motors.forEach { motor: DcMotorEx ->
            val motorConfigurationType = motor.motorType.clone()
            motorConfigurationType.achieveableMaxRPMFraction = 1.0
            motor.motorType = motorConfigurationType
        }

        lf.direction = DcMotorSimple.Direction.REVERSE
        lb.direction = DcMotorSimple.Direction.REVERSE

        phoneCam.openCameraDeviceAsync {
            phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
        }

        val deliveryZone = RingStackPipeline.deliveryZone

        waitForStart()


        forwardForTime(1500)

        when(deliveryZone){

            RingStackPipeline.DeliveryZone.A -> {
                forwardForTime(2000)
                backwardForTime(1500)
            }

            RingStackPipeline.DeliveryZone.B -> {
                turnToAngle(20.0)
                forwardForTime(2000)
                backwardForTime(2000)
            }

            RingStackPipeline.DeliveryZone.C -> {
                turnToAngle(25.0)
                forwardForTime(1500)
                backwardForTime(1500)
            }

        }

    }

    private fun stopDrive(){
        lf.power = 0.0
        lb.power = 0.0
        rf.power = 0.0
        rb.power = 0.0
    }

    private fun forwardForTime(ms: Long){
        lf.power = forwardPower
        lb.power = forwardPower
        rf.power = forwardPower
        rb.power = forwardPower
        sleep(ms)
        stopDrive()
    }

    private fun backwardForTime(ms: Long){
        lf.power = backwardPower
        lb.power = backwardPower
        rf.power = backwardPower
        rb.power = backwardPower
        sleep(ms)
        stopDrive()
    }

    private fun turnToAngle(targetAngle: Double){
        timer.reset()
        var error: Double
        while(timer.seconds() < 3.0){
            error = targetAngle - imu.angularOrientation.firstAngle
            rf.power = error * turnCoefficient
            rb.power = error * turnCoefficient
        }
        stopDrive()

    }

}