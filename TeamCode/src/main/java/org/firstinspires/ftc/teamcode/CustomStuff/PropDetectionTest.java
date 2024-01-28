package org.firstinspires.ftc.teamcode.CustomStuff;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CustomStuff.testvision.pipeline;
import org.firstinspires.ftc.teamcode.CustomStuff.vision.PropDetectionPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
@TeleOp(name = "butt")
@Config
public class PropDetectionTest extends LinearOpMode {
    public static int hsvLow1 = 36;
    public static int hsvLow2 = 0;
    public static int hsvLow3 = 0;
    public static int hsvHigh1 = 86;
    public static int hsvHigh2 = 255;
    public static int hsvHigh3 = 255;

    private VisionPortal vp1, vp2;
    private PropDetectionPipeline pdp = new PropDetectionPipeline();
    private PropDetectionPipeline pdp1 = new PropDetectionPipeline();


    @Override
    public void runOpMode () {
        pdp.isBlue = true;
        vp1 = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), pdp);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //vp = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), new pipeline());

        waitForStart();

        vp1.resumeLiveView();
        vp1.resumeStreaming();
        PropDetectionPipeline.largestContourArea = 0;
        while(opModeIsActive()) {
            telemetry.addData("Num. Contours Detected:", PropDetectionPipeline.numContours);
            telemetry.addData("LargestContourArea:", PropDetectionPipeline.largestContourArea);
            telemetry.addData("largestContourX: ", PropDetectionPipeline.largestContourX);
            telemetry.addData("larestContourY", PropDetectionPipeline.largestContourY);
            telemetry.update();


        }
    }
}
