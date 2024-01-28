package org.firstinspires.ftc.teamcode.CustomStuff.testvision;

import android.graphics.Canvas;

import com.acmerobotics.roadrunner.Math;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class pipeline implements VisionProcessor {
    private String location = "nothing"; // output

    //the color we are testing for - can be a range of colors(but only from min to max)
    /**
     * FOR SCALING SCALAR VALUES:
     * HSV stands for hue, saturation,
     * and value, hue (the 0-360° thing in google, but in openCV it's 0-180)
     * saturation (0-100% in google, but 0-255 in openCV)
     * and value (same as saturation in google/openCV)
     *
     */
    public Scalar lower = new Scalar(30, 80, 40); // HSV threshold bounds
    public Scalar upper = new Scalar(180, 200, 150);
    private Mat hsvMat = new Mat(); // converted image
    private Mat binaryMat = new Mat(); // image analyzed after thresholding
    private Mat maskedInputMat = new Mat();
    // Rectangle regions to be scanned
    private Point topLeft1 = new Point(10, 0), bottomRight1 = new Point(40, 20);
    private Point topLeft2 = new Point(10, 0), bottomRight2 = new Point(40, 20);
    private Point topLeft3 = new Point(10, 0), bottomRight3 = new Point(40, 20);
    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }
    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        // Convert from BGR to HSV
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsvMat, lower, upper, binaryMat);
        Mat balls = new Mat();
        Imgproc.Canny(binaryMat, balls, 100, 300);
        balls.copyTo(input);
        return null;
    }
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
    public String getLocation() {
        return location;
    }
}