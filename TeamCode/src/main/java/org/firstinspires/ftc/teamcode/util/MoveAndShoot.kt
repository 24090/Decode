package org.firstinspires.ftc.teamcode.util

import android.util.Log
import org.firstinspires.ftc.teamcode.opmodes.poses.inLaunchZone
import org.firstinspires.ftc.teamcode.subsystems.drive.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.Vector
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter.Params.shooterAngle
import kotlin.math.acos
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sqrt


fun moveShootKinematics(relativeVector: Vector, fieldVelocity: Vector): Pair<Double, Double>{
    val s_x: Double = relativeVector.x
    val s_y: Double = relativeVector.y
    val heightDiff: Double = 26.74534 //TODO measure height difference from shooter and goal
    val v_rx: Double = fieldVelocity.x
    val v_ry: Double = fieldVelocity.y
    val gravity = 386.088
    val t: Double = newtonQuartic(
        ((((cos(shooterAngle)).pow(2))*(gravity.pow(2)))/4)-(v_rx.pow(2)),
        2*s_y*v_rx,
        (((cos(shooterAngle)).pow(2))*heightDiff*gravity)-(v_ry.pow(2))-(s_y.pow(2)),
        2*s_x*v_ry,
        ((cos(shooterAngle)).pow(2))*(heightDiff.pow(2))-(s_x.pow(2)),
        3.0
    )
    Log.i("T: ", "$t")
    val v_s = sqrt((((-s_y+t*v_ry)/(t*Math.cos(shooterAngle))).pow(2))+(((-s_x+t*v_rx)/(cos(shooterAngle))).pow(2)))
    val phi = acos((-s_x+t*v_rx)/(v_s*t*cos(shooterAngle)))
    return Pair(v_s, phi)
}

fun calculatePredictiveMoveShoot(currentPose: Pose, fieldVelocity: Pose): Pair<Double, Double>?{
    // Start turning/spinning 5 seconds ahead
    val movementLine = Pair(currentPose.vector(), (currentPose + fieldVelocity * 5).vector())
    var closestIntersection: Vector? = null
    for (line in arrayOf(
        Pair(Vector.fromCartesian(0.0, -24.0),Vector.fromCartesian(24.0, 0.0)),
        Pair(Vector.fromCartesian(0.0, 24.0),Vector.fromCartesian(24.0, 0.0)),
        Pair(Vector.fromCartesian(72.0, 0.0),Vector.fromCartesian(0.0, 0.0)),
        Pair(Vector.fromCartesian(72.0, 0.0),Vector.fromCartesian(0.0, 0.0)),
    )) {
        val intersection = findLineIntersection(movementLine, line)
        closestIntersection = if (intersection == null){
            closestIntersection
        } else if (closestIntersection == null) {
            intersection
        } else if ((closestIntersection - currentPose.vector()).length > (intersection - currentPose.vector()).length) {
            intersection
        } else {
            closestIntersection
        }
    }
    if (closestIntersection == null) {
        return null
    }
    if (inLaunchZone(currentPose)) {
        closestIntersection = currentPose.vector()
    }
    return moveShootKinematics(Vector.fromCartesian(144.0, 72.0) - closestIntersection, fieldVelocity.vector())
}