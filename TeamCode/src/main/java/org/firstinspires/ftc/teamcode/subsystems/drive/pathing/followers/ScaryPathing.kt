package org.firstinspires.ftc.teamcode.subsystems.drive.pathing.followers

import android.util.Log
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.tipAccelBackward
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.tipAccelForward
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveVectors
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveVectors.Companion.processTurnTranslational
import org.firstinspires.ftc.teamcode.subsystems.drive.Localizer
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.minStopDistance


fun getScaryPathing(targetPose: Pose, localizer: Localizer) = { scaryPathing(localizer.pose, localizer.poseVel, targetPose) }
fun scaryPathing(pose: Pose, velocity: Pose, targetPose: Pose): DriveVectors{
    // if requiredDeaccel < maxDeaccel: go at max accel
    // when requiredDeaccel = maxDeaccel: go at max deaccel
    val error = (targetPose - pose)

    if (error.vector().length < 5.0){
        return pointToPoint(pose, velocity, targetPose, true)
    }

    val minStopDistance = minStopDistance(pose.heading, error.vector().angle, velocity, tipAccelBackward, tipAccelForward)
    val tooScary = (minStopDistance) > error.vector().length

    return if (tooScary){
        android.util.Log.w("#scarypathing", "too scary!!! $error")
        println("tooScary!!! $error")
        processTurnTranslational(0.0, error.vector().norm() * -100, pose, velocity)
    } else {
        Log.w("#scarypathing", "not scary :) $error, $minStopDistance")
        processTurnTranslational(0.0, error.vector().norm() * 100, pose, velocity)
    }
}