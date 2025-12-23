package org.firstinspires.ftc.teamcode.util

import android.util.Log
import org.firstinspires.ftc.teamcode.opmodes.poses.inLaunchZone
import org.firstinspires.ftc.teamcode.subsystems.drive.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.Vector
import kotlin.math.acos
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.math.tan


fun moveShootKinematics(relativePosition: Vector, fieldVelocity: Vector): Pair<Double, Double>{
    val sx: Double = relativePosition.x
    val sy: Double = relativePosition.y
    val vrx: Double = fieldVelocity.x
    val vry: Double = fieldVelocity.y
    val heightDiff = 26.74534
    val gravity = 386.088
    val hoodAngle = 0.678

    val t: Double = solvePolynomialIA(
        interval = Interval(0.0, 10.0),
        iterations = 50,
        -(0.25*gravity.pow(2))*(tan(hoodAngle).pow(2)),
        0.0,
        vrx.pow(2)+vry.pow(2)-(heightDiff*gravity)*(tan(hoodAngle).pow(2)),
        -2*sx*vrx-2*sy*vry,
        sx.pow(2)+sy.pow(2)-(heightDiff.pow(2))*(tan(hoodAngle).pow(2))
    ).last()
    println(t)
//    Log.d("t", "$t")
//    println("COEFFICIENTS\n" +
//            "${-(0.25*gravity.pow(2))*(tan(hoodAngle).pow(2))}, " +
//            "0.0, " +
//            "${vrx.pow(2)+vry.pow(2)-(heightDiff*gravity)*(tan(hoodAngle).pow(2))}, " +
//            "${-2*sx*vrx-2*sy*vry}, " +
//            "${sx.pow(2)+sy.pow(2)-(heightDiff.pow(2))*(tan(hoodAngle).pow(2))}"
//    )
    val vs = sqrt(
        ((sy-vry*t).pow(2) + (sx-vrx*t).pow(2))/((sin(hoodAngle)*t).pow(2))
    )
    val phi = acos((sx-t*vrx)/(vs*sin(hoodAngle)*t))
    return Pair(vs, phi)
}

fun calculatePredictiveMoveShoot(minimumTime: Double, currentPose: Pose, fieldVelocity: Pose): Pair<Double, Double>?{
    val effectivePose = (currentPose + fieldVelocity * minimumTime)
    // Start turning/spinning up 5 seconds ahead
    val movementLine = Pair(
        effectivePose.vector(),
        (currentPose + fieldVelocity * 5).vector()
    )
    var closestIntersection: Vector? = null
    for (line in arrayOf(
        Pair(Vector.fromCartesian(0.0, -24.0), Vector.fromCartesian(24.0, 0.0)),
        Pair(Vector.fromCartesian(0.0, 24.0), Vector.fromCartesian(24.0, 0.0)),
        Pair(Vector.fromCartesian(72.0, 0.0), Vector.fromCartesian(144.0, 72.0)),
        Pair(Vector.fromCartesian(72.0, 0.0), Vector.fromCartesian(144.0, -72.0)),
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
    if (inLaunchZone(effectivePose)) {
        closestIntersection = effectivePose.vector()
    }
    if (closestIntersection == null) {
        return null
    }
    return moveShootKinematics(Vector.fromCartesian(144.0, 72.0) - closestIntersection, fieldVelocity.vector())
}