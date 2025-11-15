package org.firstinspires.ftc.teamcode;


/*
* All constant value for auto.
* This allows us to easily reverse auto for different sides, flipping the auto
* by inverting the values
* */
public class AutoConstants {

        //Shooter RPM for all autos
        //Original: 3250
        final int shooterRPM = 3880;
        //How far from the RPM should we be to detect that
        //a ball was launched
        final int ballDetectedThreshhold = 500;

        //All the blue auto constants.
        //Corrdinates for Vectors, Positons,
        //Poses and values reguarding RPM, etc.
        //Red equivalent such as "red_shootingPosition_x" will
        //be equal to the negative of these.
        final int blue_towardsOblesk_shootingPosition_x = 5; //95
        final int blue_towardsOblesk_shootingPosition_y = 5; //5
        final double blue_towardsOblesk_firingPositionRotation = 30.0; //52.0
        final int blue_towardsOblesk_parkPosition_x = 30;
        final int blue_towardsOblesk_parkPosition_y = 25;


        //All the red auto constants.
        //Corrdinates for Vectors, Positons,
        //Poses and values reguarding RPM, etc.
        //Red equivalent such as "red_shootingPosition_x" will
        //be equal to the negative of the blue values.
        final int red_towardsOblesk_shootingPosition_x = blue_towardsOblesk_shootingPosition_x;
        final int red_towardsOblesk_shootingPosition_y = -blue_towardsOblesk_shootingPosition_y;
        final double red_towardsOblesk_firingPositionRotation = -(blue_towardsOblesk_firingPositionRotation + 4.0); //Adds 4 due to the shooter not being centered
        final int red_towardsOblesk_parkPosition_x = blue_towardsOblesk_parkPosition_x;
        final int red_towardsOblesk_parkPosition_y = -blue_towardsOblesk_parkPosition_y;
}
