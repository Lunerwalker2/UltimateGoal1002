package org.firstinspires.ftc.teamcode.Vision

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.Vision.pipelines.FtclibVisionTesting
import org.firstinspires.ftc.teamcode.Vision.pipelines.RingStackPipeline
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvInternalCamera2

@TeleOp
class VisionTesting(): LinearOpMode() {

    val dashboard = FtcDashboard.getInstance()

    lateinit var webcam: OpenCvInternalCamera2

    val pipeline = RingStackPipeline()

    override fun runOpMode() {

        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName)
        webcam = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.FRONT, cameraMonitorViewId)


        webcam.setPipeline(pipeline)

        webcam.openCameraDeviceAsync {
            webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
        }

        dashboard.startCameraStream(webcam, 30.0)

        waitForStart()

        while(opModeIsActive()){

            telemetry.addData("Frame Count", webcam.frameCount)
            telemetry.addData("FPS", String.format("%.2f", webcam.fps))
            telemetry.addData("Total frame time ms", webcam.totalFrameTimeMs)
            telemetry.addData("Pipeline time ms", webcam.pipelineTimeMs)
            telemetry.addData("Overhead time ms", webcam.overheadTimeMs)
            telemetry.addData("Theoretical max FPS", webcam.currentPipelineMaxFps)
            telemetry.addData("Zone", RingStackPipeline.deliveryZone)
            telemetry.update();

        }

        dashboard.stopCameraStream()
        webcam.stopStreaming()
        webcam.closeCameraDevice()

    }


}