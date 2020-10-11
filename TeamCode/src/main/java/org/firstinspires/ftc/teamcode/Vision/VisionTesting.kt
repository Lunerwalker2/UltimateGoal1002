package org.firstinspires.ftc.teamcode.Vision

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Vision.pipelines.FtclibVisionTesting
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvInternalCamera2

@TeleOp
class VisionTesting(): LinearOpMode() {


    lateinit var phoneCam: OpenCvCamera

    val pipeline = FtclibVisionTesting()

    override fun runOpMode() {

        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName)
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId)


        phoneCam.setPipeline(pipeline)

        phoneCam.openCameraDeviceAsync {
            phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
        }



        waitForStart()

        while(opModeIsActive()){

            telemetry.addData("Frame Count", phoneCam.frameCount)
            telemetry.addData("FPS", String.format("%.2f", phoneCam.fps))
            telemetry.addData("Total frame time ms", phoneCam.totalFrameTimeMs)
            telemetry.addData("Pipeline time ms", phoneCam.pipelineTimeMs)
            telemetry.addData("Overhead time ms", phoneCam.overheadTimeMs)
            telemetry.addData("Theoretical max FPS", phoneCam.currentPipelineMaxFps)
            telemetry.addData("Top Average", pipeline.topAverage)
            telemetry.addData("Bottom Average", pipeline.bottomAverage)
            telemetry.update();

        }

        phoneCam.stopStreaming()
        phoneCam.closeCameraDevice()

    }


}