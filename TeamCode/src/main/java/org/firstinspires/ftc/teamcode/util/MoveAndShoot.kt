package org.firstinspires.ftc.teamcode.util

import org.firstinspires.ftc.teamcode.opmodes.poses.inLaunchZone
import org.firstinspires.ftc.teamcode.opmodes.poses.robotLength
import org.firstinspires.ftc.teamcode.opmodes.poses.robotWidth
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Vector
import kotlin.math.PI
import kotlin.math.acos
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.math.tan

/**
@return vs, phi
 */
fun moveShootKinematics(relativePosition: Vector, fieldVelocity: Vector): Pair<Double, Double>{
    val sx: Double = relativePosition.x
    val sy: Double = relativePosition.y
    val vrx: Double = fieldVelocity.x
    val vry: Double = fieldVelocity.y
    val heightDiff = 27.17
    val gravity = 386.088
    val hoodAngle = 0.52

    val polynomial = Polynomial(
        sx.pow(2)+sy.pow(2)-(heightDiff.pow(2))*(tan(hoodAngle).pow(2)),
        -2*sx*vrx-2*sy*vry,
        vrx.pow(2)+vry.pow(2)-(heightDiff*gravity)*(tan(hoodAngle).pow(2)),
        0.0,
        -(0.25*gravity.pow(2))*(tan(hoodAngle).pow(2)),
    )

    val t: Double =
        try { polynomial.solve(Interval(0.0, 5.0)).last() }
        catch (_: NoSuchElementException) {
            polynomial.minimize(Interval(0.0, 5.0)).first()
        }

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

fun getLaunchZoneIntersections(movementLine: Line, tolerance: Double = robotWidth/2.0): List<Vector> {
    return arrayOf(
        Line(Vector.fromCartesian(0.0, -24.0 - tolerance), Vector.fromCartesian(24.0, 0.0)),
        Line(Vector.fromCartesian(0.0, 24.0 - tolerance), Vector.fromCartesian(24.0, 0.0)),
        Line(Vector.fromCartesian(72.0, 0.0), Vector.fromCartesian(144.0, 72.0)),
        Line(Vector.fromCartesian(72.0, 0.0), Vector.fromCartesian(144.0, -72.0)),
    ).mapNotNull { line -> line.lineIntersection(movementLine) }
}

/**
@return vs, phi
 */
fun calculatePredictiveMoveShoot(minimumTime: Double, currentPose: Pose, fieldVelocity: Pose, estimatedAcceleration: Pose): Pair<Double, Double>?{
    val effectivePose = (currentPose + fieldVelocity * minimumTime + estimatedAcceleration * minimumTime.pow(2)/2)
    // Start turning/spinning up 5 seconds ahead
    val movementStart = effectivePose.vector()
    val movementEnd = (currentPose + fieldVelocity * 5 + estimatedAcceleration * 25/2).vector()
    val closestIntersection: Vector? = if (inLaunchZone(effectivePose)) {
        effectivePose.vector()
    } else { (
        getLaunchZoneIntersections(
            (Vector.fromPolar(effectivePose.heading, robotLength/2) + Vector.fromPolar(effectivePose.heading + PI/2, robotLength/2))
            .let{Line(movementStart + it, movementEnd + it)}
        ) + getLaunchZoneIntersections(
            (Vector.fromPolar(effectivePose.heading, robotLength/2) + Vector.fromPolar(effectivePose.heading + PI/2, -robotLength/2))
                .let{Line(movementStart + it, movementEnd + it)}
        ) + getLaunchZoneIntersections(
            (Vector.fromPolar(effectivePose.heading, -robotLength/2) + Vector.fromPolar(effectivePose.heading + PI/2, robotLength/2))
                .let{Line(movementStart + it, movementEnd + it)}
        ) + getLaunchZoneIntersections(
            (Vector.fromPolar(effectivePose.heading, -robotLength/2) + Vector.fromPolar(effectivePose.heading + PI/2, -robotLength/2))
                .let{Line(movementStart + it, movementEnd + it)}
        )
    ).reduceOrNull { a, b -> if ((a - effectivePose.vector()).length < (b - effectivePose.vector()).length) a else b } }
    if (closestIntersection == null) {
        return null
    }
    return moveShootKinematics(Vector.fromCartesian(144.0, 72.0) - closestIntersection, fieldVelocity.vector())
}