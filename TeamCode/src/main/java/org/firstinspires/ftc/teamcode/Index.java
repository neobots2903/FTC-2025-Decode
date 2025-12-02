package org.firstinspires.ftc.teamcode;

/*
* This class represents an
* index, holding data reguarding
* the status of whats in the index and where it is
* */
public class Index {

    //ENUM defines the possible states
    //we could have at any index in the indexer.
    //------
    //Ball States:
    //
    //GREEN -> A green ball is in the index
    //
    //PURPLE -> A purple ball is in the index
    //
    //EMPTY -> Nothing is in the index
    //
    //UNKNOWN -> We don't know whats in the index?
    enum BallStates {GREEN, PURPLE, EMPTY, UNKNOWN}

    //The current ball state of the index
    //Is the ball green, purple, unknown, empty, etc.
    BallStates ballState;

    //The position of the indexer and where
    //the index is in the cycle
    //-------
    //Positions:
    //
    //LAUNCH -> The ball is at the top of the index by the launcher
    //
    //INTAKE -> The ball is at the bottom by the intake
    //
    //OFFHAND -> The ball is off to the side away from the intake or launcher
    enum Position {LAUNCH, INTAKE, OFFHAND}

    //The current position of the index
    //in the index cycle (LAUNCH, INTAKE, ETC)
    Position currentPosition;

}
