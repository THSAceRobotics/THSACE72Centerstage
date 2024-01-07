package org.firstinspires.ftc.teamcode.CustomStuff;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.*;

public class AprilTagLocalizer {
    private final double TOP_RIGHT_X = 72;
    private final double TOP_LEFT_X = 72;
    private final double BOTTOM_LEFT_X = -72;
    private final double BOTTOM_RIGHT_X = -72;
    private final double TOP_RIGHT_Y = 72;
    private final double TOP_LEFT_Y = -72;
    private final double BOTTOM_LEFT_Y = -72;
    private final double BOTTOM_RIGHT_Y = 72;
    private final double TOP_LENGTH = 144;
    private final double BOTTOM_LENGTH = 144;
    private final double CAM_TO_POINT_OFFSET = 1;
    private final double POINT_TO_CENTER_OFFSET = 6.35;
    private final double ROBOT_CAMERA_OFFSET = 180;
    private final boolean isAbove = true;

    private VisionPortal vp;
    private AprilTagProcessor apt;

    SampleMecanumDrive drive;
    public AprilTagLocalizer(HardwareMap hwMap, SampleMecanumDrive drive) {
        this.drive = drive;
        apt = AprilTagProcessor.easyCreateWithDefaults();
        vp = VisionPortal.easyCreateWithDefaults(hwMap.get(WebcamName.class,"Webcam 1"), apt);
    }

    public void startStream() {
        vp.resumeStreaming();
    }

    public void stopStream() {
        vp.stopStreaming();
    }

    public double[] getPos() {
        vp.resumeStreaming();
        ArrayList<AprilTagDetection> wCam = null;
        while (wCam == null)
            wCam = apt.getFreshDetections();
        vp.stopStreaming();

        double robotCurrentX = drive.getPoseEstimate().getX();
        double robotCurrentY = drive.getPoseEstimate().getY();
        double tag1X, tag1Y, tag2X, tag2Y, r1, r2, tag1Bearing, tag2Bearing;

        if(robotCurrentY < 0) {
            wCam.removeIf(a -> a.id < 4 || a.id == 9 || a.id == 10);
        }

        if(robotCurrentY > 0) {
            wCam.removeIf(a -> a.id == 4 || a.id == 5 || a.id == 6 || a.id == 7 || a.id == 8);
        }

        ArrayList<Double> xVals = new ArrayList<Double>();
        ArrayList<Double> yVals = new ArrayList<Double>();
        ArrayList<Double> headingVals = new ArrayList<Double>();

        for(int i = 0; i < wCam.size(); i++) {
            tag1X = wCam.get(i).metadata.fieldPosition.get(0);
            tag1Y = wCam.get(i).metadata.fieldPosition.get(1);
            tag1Bearing = wCam.get(i).ftcPose.bearing * 180 / Math.PI;
            r1 = wCam.get(i).ftcPose.range;
            for(int j = 0; j < wCam.size(); j++) {
                tag2X = wCam.get(i).metadata.fieldPosition.get(0);
                tag2Y = wCam.get(i).metadata.fieldPosition.get(1);
                tag2Bearing = wCam.get(i).ftcPose.bearing * 180 / Math.PI;
                r2 = wCam.get(i).ftcPose.range;
                double modifiedTag2X = tag2X + -1 * tag1X;
                double modifiedTag2Y = tag2Y + -1 * tag1Y;
                double x2;
                double y = 2 * -modifiedTag2Y;
                double x = 2 * -modifiedTag2X;
                double c = -modifiedTag2X * -modifiedTag2X + -modifiedTag2Y * -modifiedTag2Y - r2 * r2;
                x = -x/y;
                c = -(c + r1 * r1)/y;
                x2 = x * x;
                x = 2 * x * c;
                c = c * c;
                c = c - r1 * r1;
                x2++;
                double ans1x = (-x + Math.sqrt(x * x - 4 * x2 * c)) / (2 * x2);
                double ans2x = (-x - Math.sqrt(x * x - 4 * x2 * c)) / (2 * x2);
                double ans1y = Math.sqrt(Math.pow(r1, 2) - Math.pow(ans1x, 2));
                double ans2y = Math.sqrt(Math.pow(r1, 2) - Math.pow(ans2x, 2));
                ans1x = ans1x + tag1X;
                ans2x = ans2x + tag1X;
                ans1y = ans1y + tag1Y;
                ans2y = ans2y + tag1Y;
                double robotX;
                double robotY;

                if(robotCurrentX - ans1x < robotCurrentX - ans2x && robotCurrentY - ans1y < robotCurrentY - ans2y) {
                    robotX = ans1x;
                    robotY = ans1y;
                } else {
                    robotX = ans2x;
                    robotY = ans2y;
                }

                double sideBetweenTags = Math.sqrt(Math.pow((Math.max(tag1X, tag2X)) - (Math.min(tag1X, tag2X)), 2) + Math.pow((Math.max(tag1Y, tag2Y)) - (Math.min(tag1Y, tag2Y)), 2));
                double sideBetweenRobotAndTag1 = Math.sqrt(Math.pow((Math.max(robotX, tag1X)) - (Math.min(robotX, tag1X)), 2) + Math.pow((Math.max(robotY, tag1Y)) - (Math.min(robotY, tag1Y)), 2));
                double sideBetweenRobotAndTag2 = Math.sqrt(Math.pow((Math.max(robotX, tag2X)) - (Math.min(robotX, tag2X)), 2) + Math.pow((Math.max(robotY, tag2Y)) - (Math.min(robotY, tag2Y)), 2));

                double chosenTagX = 0, chosenTagY = 0, chosenTagBearing = 0;

                if(tag1Y == TOP_LEFT_Y && tag2Y == TOP_LEFT_Y) {
                    if((Math.min(tag1X, tag2X)) < (Math.max(tag1X, tag2X)) && (Math.max(tag1X, tag2X)) < (TOP_LENGTH / 2 + TOP_LEFT_X)) {
                        chosenTagX = Math.min(tag1X, tag2X);
                        chosenTagY = TOP_LEFT_Y;
                        chosenTagBearing = (chosenTagX == tag1X) ? tag1Bearing : tag2Bearing;
                    } else if((TOP_LENGTH / 2 + TOP_LEFT_X) < (Math.min(tag1X, tag2X)) && (Math.min(tag1X, tag2X)) < (Math.max(tag1X, tag2X))) {
                        chosenTagX = Math.max(tag1X, tag2X);
                        chosenTagY = TOP_LEFT_Y;
                        chosenTagBearing = (chosenTagX == tag1X) ? tag1Bearing : tag2Bearing;
                    } else {
                        chosenTagX = tag1X;
                        chosenTagY = TOP_LEFT_Y;
                        chosenTagBearing = tag1Bearing;
                    }
                } else if(tag1Y == BOTTOM_RIGHT_Y && tag2Y == BOTTOM_RIGHT_Y) {
                    if((Math.min(tag1X, tag2X)) < (Math.max(tag1X, tag2X)) && (Math.max(tag1X, tag2X)) < (BOTTOM_LENGTH / 2 + BOTTOM_LEFT_X)) {
                        chosenTagX = Math.min(tag1X, tag2X);
                        chosenTagY = BOTTOM_LEFT_Y;
                        chosenTagBearing = (chosenTagX == tag1X) ? tag1Bearing : tag2Bearing;
                    } else if((BOTTOM_LENGTH / 2 + BOTTOM_LEFT_X) < (Math.min(tag1X, tag2X)) && (Math.min(tag1X, tag2X)) < (Math.max(tag1X, tag2X))) {
                        chosenTagX = Math.max(tag1X, tag2X);
                        chosenTagY = BOTTOM_LEFT_Y;
                        chosenTagBearing = (chosenTagX == tag1X) ? tag1Bearing : tag2Bearing;
                    } else {
                        chosenTagX = tag1X;
                        chosenTagY = BOTTOM_LEFT_Y;
                        chosenTagBearing = tag1Bearing;
                    }
                }

                double tagAngle = 0;

                if(chosenTagX == tag2X && chosenTagY == tag2Y)
                    tagAngle = (Math.acos((Math.pow(sideBetweenRobotAndTag1, 2) - Math.pow(sideBetweenTags, 2) - Math.pow(sideBetweenRobotAndTag2, 2)) / (-2 * sideBetweenTags * sideBetweenRobotAndTag2))) * (180 / Math.PI);
                else if(chosenTagX == tag1X && chosenTagY == tag1Y)
                    tagAngle = (Math.acos((Math.pow(sideBetweenRobotAndTag2, 2) - Math.pow(sideBetweenTags, 2) - Math.pow(sideBetweenRobotAndTag1, 2)) / (-2 * sideBetweenTags * sideBetweenRobotAndTag1))) * (180 / Math.PI);

                double robotAngle = 0;

                if(tag2X > robotX && tag2Y > robotY) {
                    robotAngle = tagAngle + chosenTagBearing + ROBOT_CAMERA_OFFSET; //VARIABLE NEEDS TO BE DECLARE
                } else if(tag2X > robotX && tag2Y < robotY) {
                    robotAngle = 180 - tagAngle + chosenTagBearing + ROBOT_CAMERA_OFFSET;
                } else if(tag2X < robotX && tag2Y < robotY) {
                    robotAngle = 180 + tagAngle + chosenTagBearing + ROBOT_CAMERA_OFFSET;
                } else if(tag2X < robotX && tag2Y > robotY) {
                    robotAngle = 360 - tagAngle + chosenTagBearing + ROBOT_CAMERA_OFFSET;
                }

                headingVals.add(robotAngle);

                double m = Math.tan(robotAngle - ROBOT_CAMERA_OFFSET);
                double a = 1 + m * m;
                double b = (2 * -robotX) + ((m * -robotX) * m * 2);
                c = Math.pow(-robotX, 2) + Math.pow(m * -robotX, 2) - Math.pow(CAM_TO_POINT_OFFSET, 2);

                double pointx1 = (-b + Math.sqrt(b * b - 4 * a * c)) / 2 * a;
                double pointx2 = (-b - Math.sqrt(b * b - 4 * a * c)) / 2 * a;
                double pointy1 = m * (pointx1 - robotX) + robotY;
                double pointy2 = m * (pointx2 - robotX) + robotY;
                double chosenPointX = 0, chosenPointY = 0;

                if(robotAngle - ROBOT_CAMERA_OFFSET < 90 || (270 < robotAngle - ROBOT_CAMERA_OFFSET && robotAngle - ROBOT_CAMERA_OFFSET <= 360)) {
                    chosenPointX = Math.min(pointx1, pointx2);
                    if (chosenPointX == pointx1) {
                        chosenPointY = pointy1;
                    } else if (chosenPointX == pointx2) {
                        chosenPointY = pointy2;
                    }
                } else if (robotAngle - ROBOT_CAMERA_OFFSET == 90) {
                    chosenPointY = Math.min(pointy1, pointy2);
                    if (chosenPointY == pointy1) {
                        chosenPointX = pointx1;
                    } else if (chosenPointY == pointy2) {
                        chosenPointX = pointx2;
                    }
                } else if (robotAngle - ROBOT_CAMERA_OFFSET == 270) {
                    chosenPointY = Math.max(pointy1, pointy2);
                    if (chosenPointY == pointy1) {
                        chosenPointX = pointx1;
                    } else if (chosenPointY == pointy2) {
                        chosenPointX = pointx2;
                    }
                } else {
                    chosenPointX = Math.max(pointx1, pointx2);
                    if (chosenPointX == pointx1) {
                        chosenPointY = pointy1;
                    } else if (chosenPointX == pointx2) {
                        chosenPointY = pointy2;
                    }
                }

                m = Math.tan(robotAngle - ROBOT_CAMERA_OFFSET);
                a = 1 + m * m;
                b = (2 * -chosenPointX) + ((m * -chosenPointX) * m * 2);
                c = Math.pow(-chosenPointX, 2) + Math.pow(m * -chosenPointX, 2) - Math.pow(POINT_TO_CENTER_OFFSET, 2);

                pointx1 = (-b + Math.sqrt(b * b - 4 * a * c)) / 2 * a;
                pointx2 = (-b - Math.sqrt(b * b - 4 * a * c)) / 2 * a;
                pointy1 = m * (pointx1 - chosenPointX) + chosenPointY;
                pointy2 = m * (pointx2 - chosenPointX) + chosenPointY;

               if(isAbove) {
                   if(robotAngle < 90 || (270 < robotAngle && robotAngle <= 360)) {
                       chosenPointX = Math.max(pointx1, pointx2);
                       if (chosenPointX == pointx1) {
                           chosenPointY = pointy1;
                       } else if (chosenPointX == pointx2) {
                           chosenPointY = pointy2;
                       }
                   } else if (robotAngle == 90) {
                       chosenPointY = Math.max(pointy1, pointy2);
                       if (chosenPointY == pointy1) {
                           chosenPointX = pointx1;
                       } else if (chosenPointY == pointy2) {
                           chosenPointX = pointx2;
                       }
                   } else if (robotAngle == 270) {
                       chosenPointY = Math.min(pointy1, pointy2);
                       if (chosenPointY == pointy1) {
                           chosenPointX = pointx1;
                       } else if (chosenPointY == pointy2) {
                           chosenPointX = pointx2;
                       }
                   } else {
                       chosenPointX = Math.min(pointx1, pointx2);
                       if (chosenPointX == pointx1) {
                           chosenPointY = pointy1;
                       } else if (chosenPointX == pointx2) {
                           chosenPointY = pointy2;
                       }
                   }
               } else {
                   if (robotAngle < 90 || (270 < robotAngle && robotAngle <= 360)) {
                       chosenPointX = Math.min(pointx1, pointx2);
                       if (chosenPointX == pointx1) {
                           chosenPointY = pointy1;
                       } else if (chosenPointX == pointx2) {
                           chosenPointY = pointy2;
                       }
                   } else if (robotAngle == 90) {
                       chosenPointY = Math.min(pointy1, pointy2);
                       if (chosenPointY == pointy1) {
                           chosenPointX = pointx1;
                       } else if (chosenPointY == pointy2) {
                           chosenPointX = pointx2;
                       }
                   } else if (robotAngle == 270) {
                       chosenPointY = Math.max(pointy1, pointy2);
                       if (chosenPointY == pointy1) {
                           chosenPointX = pointx1;
                       } else if (chosenPointY == pointy2) {
                           chosenPointX = pointx2;
                       }
                   } else {
                       chosenPointX = Math.max(pointx1, pointx2);
                       if (chosenPointX == pointx1) {
                           chosenPointY = pointy1;
                       } else if (chosenPointX == pointx2) {
                           chosenPointY = pointy2;
                       }
                   }
               }
            }
        }
        double x = 0, y = 0, heading = 0;

        for (double i : xVals)
            x += i;
        x = x / xVals.size();

        for (double i : yVals)
            y += i;
        y = y / yVals.size();

        for (double i : headingVals)
            heading += i;
        heading = heading / headingVals.size();

        return new double [] {x, y, heading * Math.PI / 180};

    }
}

