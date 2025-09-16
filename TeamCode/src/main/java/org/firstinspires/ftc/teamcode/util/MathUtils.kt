package org.firstinspires.ftc.teamcode.util

import kotlin.math.max
import kotlin.math.min

fun Int.factorial() : Int = (1..this).reduce(Int::times)
/**
 * @return The number of ways to choose `r` unique elements from a set of `n` without replacement, ignoring order
 */
fun nCr(n: Int, r: Int) = n.factorial()/(r.factorial() * (n-r).factorial())
/**
 * Infix version of `nCr`
 * @see nCr
 */
infix fun Int.choose(r: Int) = nCr(this, r)

fun clamp(x: Double, min: Double, max: Double): Double {
    assert(min <= max)
    return min(max(x, min), max)
}