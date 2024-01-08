package org.firstinspires.ftc.teamcode.CustomStuff;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CustomStuff.vision.PropDetectionPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
@TeleOp
public class PropDetectionTest extends LinearOpMode {
    public static int hsvLow1 = 36;
    public static int hsvLow2 = 0;
    public static int hsvLow3 = 0;
    public static int hsvHigh1 = 86;
    public static int hsvHigh2 = 255;
    public static int hsvHigh3 = 255;

    private VisionPortal vp;
    private PropDetectionPipeline pdp = new PropDetectionPipeline();

    @Override
    public void runOpMode () {
        vp = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), pdp);

        waitForStart();

        vp.resumeLiveView();
        vp.resumeStreaming();

        while(opModeIsActive()) {
            telemetry.addData("Num. Contours Detected:", PropDetectionPipeline.numContours);
            telemetry.addData("LargestContourArea:", PropDetectionPipeline.largestContourArea);
            telemetry.addData("largestContourX: ", PropDetectionPipeline.largestContourX);
            telemetry.addData("larestContourY", PropDetectionPipeline.largestContourY);
        }
    }
}
