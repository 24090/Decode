package org.firstinspires.ftc.teamcode.commands

class WaitUntil(val f: () -> Boolean, name: String = "WaitUntil"): OverrideButtonCommand(name) {
    override fun run(): CommandResult {
        if (!f()){
            return CommandResult.Continue
        }
        return CommandResult.End(Result.success("WaitUntil ended"))
    }
}