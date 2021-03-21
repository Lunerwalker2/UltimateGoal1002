package org.firstinspires.ftc.teamcode.Vision

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.Telemetry
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

    lateinit var camera: OpenCvCamera

    val pipeline = RingStackPipeline()

    override fun runOpMode() {

        telemetry = MultipleTelemetry(telemetry, dashboard.telemetry)

        val webcamName: WebcamName = hardwareMap.get(WebcamName::class.java, "Webcam 1")

        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName)
//        camera = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId)
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId)



        camera.setPipeline(pipeline)


        camera.openCameraDeviceAsync {
            camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
        }

        dashboard.startCameraStream(camera, 30.0)

        waitForStart()

        while(opModeIsActive()){

            telemetry.addData("Frame Count", camera.frameCount)
            telemetry.addData("FPS", String.format("%.2f", camera.fps))
            telemetry.addData("Total frame time ms", camera.totalFrameTimeMs)
            telemetry.addData("Pipeline time ms", camera.pipelineTimeMs)
            telemetry.addData("Overhead time ms", camera.overheadTimeMs)
            telemetry.addData("Theoretical max FPS", camera.currentPipelineMaxFps)
            telemetry.addData("Zone", RingStackPipeline.deliveryZone)
            telemetry.update();

        }

        dashboard.stopCameraStream()
        camera.stopStreaming()
        camera.closeCameraDevice()

    }


}