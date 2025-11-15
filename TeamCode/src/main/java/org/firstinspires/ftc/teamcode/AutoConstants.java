package org.firstinspires.ftc.teamcode;


/*
* All constant value for auto.
* This allows us to easily reverse auto for different sides, flipping the auto
* by inverting the values
* */
public class AutoConstants {

        //Shooter RPM for all autos
        final int shooterRPM = 3250;

        //All the blue auto constants.
        //Corrdinates for Vectors, Positons,
        //Poses and values reguarding RPM, etc.
        //Red equivalent such as "red_shootingPosition_x" will
        //be equal to the negative of these.
        final int blue_towardsOblesk_shootingPosition_x = 95;
        final int blue_towardsOblesk_shootingPosition_y = -5;
        final double blue_towardsOblesk_firingPositionRotation = 57.0;
        final int blue_towardsOblesk_parkPosition_x = 30;
        final int blue_towardsOblesk_parkPosition_y = 25;


        //All the red auto constants.
        //Corrdinates for Vectors, Positons,
        //Poses and values reguarding RPM, etc.
        //Red equivalent such as "red_shootingPosition_x" will
        //be equal to the negative of the blue values.
        final int red_towardsOblesk_shootingPosition_x = blue_towardsOblesk_shootingPosition_x;
        final int red_towardsOblesk_shootingPosition_y = -blue_towardsOblesk_shootingPosition_y;
        final double red_towardsOblesk_firingPositionRotation = -blue_towardsOblesk_firingPositionRotation;
        final int red_towardsOblesk_parkPosition_x = blue_towardsOblesk_parkPosition_x;
        final int red_towardsOblesk_parkPosition_y = -blue_towardsOblesk_parkPosition_y;
}
