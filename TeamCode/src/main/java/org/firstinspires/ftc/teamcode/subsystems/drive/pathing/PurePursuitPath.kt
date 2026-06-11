package org.firstinspires.ftc.teamcode.subsystems.drive.pathing

import android.util.Log
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.followers.HeadingBehaviour
import org.firstinspires.ftc.teamcode.util.Line
import org.firstinspires.ftc.teamcode.util.clamp
import kotlin.math.abs
import kotlin.math.min

class PurePursuitPath(val poses: List<Pose>, val headingBehaviours: List<HeadingBehaviour>, val maxRadii: List<Double>) {
    val points = poses.map(Pose::vector)
    var t = 0.0
    val lines = points
        .subList(0, points.size - 1)
        .zip(points.subList(1, points.size))
        .map { Line(it.first, it.second) }

    fun adjustT(t: Double): Double{
        val t = clamp(t, 0.0, lines.size.toDouble() - 1e-10)
        return (0..< t.toInt()).sumOf { lines[it].length } + lines[t.toInt()].length * (t%1)
    }

    fun unadjustT(adjustedT: Double): Double{
        var adjustedT = adjustedT
        var t = 0.0
        for (line in lines){
            val step = min(line.length, adjustedT)
            t += step/line.length
            adjustedT -= step
            if (adjustedT < 0.01) break
        }
        return t
    }

    fun maxRadius(t: Double) = maxRadii[clamp(t, 0.0, maxRadii.size.toDouble() - 1).toInt()]
    fun headingBehaviour(t: Double) = headingBehaviours[clamp(t, 0.0, maxRadii.size.toDouble() - 1).toInt()]

    fun getTValue(point: Vector, lastT: Double, tThresh: Double): Double{
        return lines
            .mapIndexed { index, line ->
                val projectedPoint = line.projectPoint(point)
                index.toDouble() + clamp(line.getTvalue(projectedPoint), 0.0, 1 - 1e-5)
            }
            .filter { abs(adjustT(it) - adjustT(lastT)) <= tThresh }
            .minBy { (getPosition(it) - point).length }
            ?: lastT
    }

    fun getFollowPoint(currentPosition: Vector, t: Double, tThresh: Double): Pair<Double, Vector>{
        val options =  lines
            .mapIndexed { index, line ->
                line.circleIntersection(
                    currentPosition,
                    maxRadius(t)
                ).map { Pair(
                    index.toDouble() + line.getTvalue(it),
                    it
                )}
            }
            .reduce{ a, b -> a+b }
            .filter { abs(adjustT(it.first) - adjustT(t)) <= tThresh } +
            listOf(unadjustT(adjustT(t) + maxRadius(t)).let { Pair(it - 0.001, getPosition(it - 0.001)) }).also {
            }



        return options.maxBy { it.first }
    }

    fun getFollowPose(currentPosition: Vector, tThresh: Double): Pose {
        t = getTValue(currentPosition, t, tThresh)
        if ((t > (lines.size-1))&&((currentPosition - points.last()).length < maxRadius(t) * 2.0)){
            return poses.last()
        }
        val (followT, followPoint) = getFollowPoint(currentPosition, t,  tThresh)
        Log.i("followT", "$followT")
        val headingBehaviour = headingBehaviour(t)
        val heading = when (headingBehaviour){
            is HeadingBehaviour.Tangent -> AngleUnit.normalizeRadians((followPoint - currentPosition).angle + headingBehaviour.angle)
            HeadingBehaviour.Snap -> poses[t.toInt() + 1].heading
            HeadingBehaviour.Interpolate -> {
                val headingT = clamp(followT - t.toInt().toDouble(), 0.0, 1.0)
                poses[t.toInt()].heading * (1 - headingT) + poses[t.toInt() + 1].heading * headingT
            }
        }

        return Pose(followPoint.x, followPoint.y, heading)
    }

    fun getPosition(t: Double) = lines[t.toInt()].getPoint(t%1)
}