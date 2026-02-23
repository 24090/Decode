package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.util.RobotLog.v
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Vector
import kotlin.math.abs
import kotlin.math.sign
import kotlin.math.sqrt

class Line(val start: Vector, val end: Vector) {
    val length
        get() = (start - end).length
    fun getTvalue(point: Vector): Double{
        if (abs(start.x - end.x)< abs(start.y - end.y)){
            if (start.y == end.y){
                return 1.0
            }
            return (point.y - start.y)/(end.y - start.y)
        } else if (start.x == end.x){
            return 1.0
        }
        return (point.x - start.x)/(end.x - start.x)
    }

    fun projectPoint(point: Vector): Vector {
        val result = (end - start).projection(point) + start
        return if (getTvalue(result) > 1) {
            end
        } else if (getTvalue(result) < 0) {
            start
        } else {
            result
        }

    }

    fun getPoint(t: Double): Vector {
        return start * (1-t) + end * t
    }


    fun circleIntersection(center: Vector, radius: Double): List<Vector> =
        Line(this.start - center, this.end - center).circleIntersection(radius).map { it + center }

    fun circleIntersection(radius: Double): List<Vector>{
        val dx = end.x - start.x
        val dy = end.y - start.y
        val dr = sqrt(dx * dx + dy * dy)
        val D = start.x * end.y - end.x * start.y
        val discriminant = radius*radius * dr*dr - D*D
        val sgn = { v: Double ->
            if (v >= 0){
                1.0
            } else {
                -1.0
            }
        }
        return if (discriminant < 0.0){
            listOf()
        } else if (discriminant == 0.0){
            println("ZERO!")
            listOf(Vector.fromCartesian(
                (D * dy)/(dr*dr),
                -(D * dx)/(dr*dr)
            ))
        } else {
            val a = Vector.fromCartesian(
                ((D * dy) + sgn(dy)*dx*sqrt(discriminant))/(dr*dr),
                (-(D * dx) +   abs(dy)*sqrt(discriminant))/(dr*dr)
            )
            val b = Vector.fromCartesian(
                ((D * dy) - sgn(dy)*dx*sqrt(discriminant))/(dr*dr),
                (-(D * dx) -   abs(dy)*sqrt(discriminant))/(dr*dr)
            )
            listOf(a, b)
        }.filter { point -> getTvalue(point).let { 0 < it && it < 1 } }
    }

    override fun toString() = "[${this.start} -> ${this.end}]"
}