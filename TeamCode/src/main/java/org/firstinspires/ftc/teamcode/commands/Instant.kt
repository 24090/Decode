package org.firstinspires.ftc.teamcode.commands

class Instant(val f: () -> Unit, name: String = "Instant"): Command(name){
    override fun run(): CommandResult {
        f.invoke()
        return CommandResult.End(Result.success("Instant function ran"))
    }
}