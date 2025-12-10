package org.firstinspires.ftc.teamcode.util

import kotlin.math.max
import kotlin.math.min
import kotlin.math.pow

class Interval(vararg values: Double) {
    val lower = values.reduce(::min)
    val upper = values.reduce(::max)

    operator fun plus(interval: Interval) = Interval(lower + interval.lower, upper + interval.upper)
    operator fun plus(value: Double) = Interval(lower + value, upper + value)
    operator fun times(interval: Interval) = Interval(lower * interval.lower, upper * interval.lower, lower * interval.upper, upper * interval.upper)
    operator fun times(value: Double) = Interval(lower * value, upper * value)

    fun pow(n: Int) = Interval(lower.pow(n), upper.pow(n))
    
    fun contains(value: Double) : Boolean {
        return lower <= value && upper >= value
    }
}