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

import java.util.ArrayList;
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
    * The opmode used for hardware map
    * and to print to the telemetry.
    * */
    private OpMode opMode;

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

        //Set the opmode so we can use telemetry,
        //etc
        this.opMode = opMode;

        //Setup the april tag processor with its default paramters
        //and settings. "The easy way"
        tagProcessor = AprilTagProcessor.easyCreateWithDefaults();

        //Setup camera one with the default vision portal (vision stream)
        //Give it an instance of the april tag processor for tag processing
        camera_one = VisionPortal.easyCreateWithDefaults(
                opMode.hardwareMap.get(WebcamName.class, "camera_one"), tagProcessor);
    }



    //Returns the distance in units to the score area
    //as an integer
    //If no tag is detected, we return 0.
    public double getDistanceToScore() {

        //The distance to the april tag
        //on the score area
        double distance = 0;

        //The two possible tags we can detect.
        //We will store an instance of detected tags here.
        AprilTagDetection possibleTagOne = null;
        AprilTagDetection possibleTagTwo = null;
        AprilTagDetection tag = null; //The tag we detected

        //Try to detect both of the tags that
        //exist in the teleop.
        possibleTagOne = findTag(20); //Look for ID 20;
        possibleTagTwo = findTag(24); //Look for ID 24;

        //Check both the detections.
        //If one of the detections exists, assign it to
        //tag so we can get distance derived from the tag.
        //If it doesn't exist, then we will return 0 distance
        //later as tag = null.
        if (possibleTagOne != null) {
            tag = possibleTagOne;
        } else if (possibleTagTwo != null) {
            tag = possibleTagTwo;
        }

        //If no tags detected, return 0
        if (tag == null) {
            return 0.0;
        }

        //We have a tag detected,
        //set the distance to the score
        //area from the tag; "distance"
        //will hold this value, return the
        //distance.
        distance = tag.ftcPose.y;

        return distance;
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


    //Returns an array containing all visible april tag
    //ids in the form of int
    //Returns "-1" if no tags found.
    public List<Integer> getAllAprilTag_ids() {

        //List of all tag ids
        List<Integer> aprilTagIds = new ArrayList<>();

        //Create a list to hold all detected april tags.
        //Take the "tagProcessor" which is "AprilTagProcessor" and return
        //instances of all tags detected by it into our list.
        //We can then get the length of this list to list of the
        //total detected tags.
        List<AprilTagDetection> currentDetections = tagProcessor.getDetections();

        //Loop through all detected april tags,
        //append each detected id onto the end of the aprilTagsIds array.
        for (AprilTagDetection detection : currentDetections) {
            aprilTagIds.add(detection.id);
        }

        //If no tags where detected, we can't return an id, so return -1
        //as a place holder.
        if (aprilTagIds.size() == 0) {
            aprilTagIds.add(-1);
        }


        //Return an array of integers, of each tag
        //id we detected.
        return aprilTagIds;
    }


    //Searchs tag detections,
    //and returns and instance of the april tag
    //by the id of argument 1 "id"
    //
    //Arguments:
    //"id" -> The id of the april tag to find.
    public AprilTagDetection findTag(int id) {

        //All april tags currently detected
        List<AprilTagDetection> currentDetections = tagProcessor.getDetections();

        //This will hold the instance of the tag we detected.
        AprilTagDetection aprilTag = null;

        //Loop through all detected april tags,
        //if the id of the tag matches to the one
        //we are searching for, then set "aprilTag" as
        //an instance of the tag.
        for (AprilTagDetection detection : currentDetections) {

            //If the tag id matches
            //the id of the tag we are searching for,
            //then set "aprilTag" the instance we return as
            //an instance of detection.
            if (detection.id == id) {
                aprilTag = detection;
            }
        }
        return aprilTag;
    }

    //Returns all april tag ids detected by
    //camera one
    public void getAllTagIds_cameraOne() {

    }
}
