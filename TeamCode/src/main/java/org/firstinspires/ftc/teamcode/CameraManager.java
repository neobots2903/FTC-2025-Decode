/*
* THis class manages all the cameras onboard the robot and allows us to interface
* them, retrieve data from them, etc.
* */
package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Collection;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;



public class CameraManager {

    /*
    * Processor for all april tags. Part of the first inspires
    * Library. Regonizes and returns the ids of april tags
    * if tags are readible from the cameras stream.
    * */
    private AprilTagProcessor tagProcessor;


    /*
    * The vision portal, which allows us to access
    * the cameras vision stream on the robot.
    * Part of the first inspires library.
    * */
    private VisionPortal camera_one;

    //Constructor
    public CameraManager(OpMode opMode) {

        //Setup "camera_one".
        initCameraOne(opMode);

    }



    /*
    * Intializes the vision portal and all parameters
    * for the first webcam, "camera_one".
    * This will allow us to use the camera and pass its feed
    * into the april tag processor and search for april tag ids
    *
    * Arguments:
    * -"opMode" -> The op mode from teleop. This contains instances of the hardware map, robot
    * parameters, etc.
    *
    * */
    private void initCameraOne(OpMode opMode) {

        //Setup the april tag processor with its default paramters
        //and settings. "The easy way"
        tagProcessor = AprilTagProcessor.easyCreateWithDefaults();

        //Setup camera one with the default vision portal (vision stream)
        //Give it an instance of the april tag processor for tag processing
        camera_one = VisionPortal.easyCreateWithDefaults(
                opMode.hardwareMap.get(WebcamName.class, "camera_one"), tagProcessor);
    }


    /*
    * Returns a 32 bit integer of the total april tags detected.
    * No other data about april tags will be returned, just the size
    * of the array/list holding all detected tags.
    * */
    public int getTotalTagsDetected() {
        int totalTagDetections = 0;

        //Create a list to hold all detected april tags.
        //Take the "tagProcessor" which is "AprilTagProcessor" and return
        //instances of all tags detected by it into our list.
        //We can then get the length of this list to list of the
        //total detected tags.
        List<AprilTagDetection> currentDetections = tagProcessor.getDetections();

        //Get the length of our detections list.
        //This is the total tags detected.
        totalTagDetections = currentDetections.size();

        return totalTagDetections;
    }


    //Returns all april tag ids detected by
    //camera one
    public void getAllTagIds_cameraOne() {

    }
}
