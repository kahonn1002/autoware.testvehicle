# Owned, bounded GUI startup for stationary_readiness.py. No remote command port.
if {![info exists env(CM_STATIONARY_GUARD)] || $env(CM_STATIONARY_GUARD) ne "verified"} {
    error "Start only through the stationary readiness harness"
}
set CMStationarySeconds $env(CM_STATIONARY_SECONDS)
set CMStationaryLogs $env(CM_STATIONARY_LOGS)
if {![string is integer -strict $CMStationarySeconds] || $CMStationarySeconds < 20 || $CMStationarySeconds > 120} {
    error "Invalid bounded test duration"
}
set CMStationaryOwnsApp 0
set CMStationaryEnding 0

proc CMStationaryExit {} {
    global CMStationaryOwnsApp
    if {$CMStationaryOwnsApp} {
        catch {puts "BACKEND_LOG_BEGIN\n[Application consoleoutput]\nBACKEND_LOG_END"}
        catch {Application stop}
    }
    exit
}

proc CMStationaryEnd {} {
    global CMStationaryEnding CMStationaryOwnsApp
    if {$CMStationaryEnding} {return}
    set CMStationaryEnding 1
    if {$CMStationaryOwnsApp} {catch {StopSim}}
    after 1500 CMStationaryExit
}

proc CMStationaryPoll {} {
    global CMStationaryLogs
    if {[file exists [file join $CMStationaryLogs stop_requested]]} {
        CMStationaryEnd
    } else {
        after 250 CMStationaryPoll
    }
}

after [expr {1000*($CMStationarySeconds+35)}] CMStationaryEnd
after 250 CMStationaryPoll
if {[catch {
    if {[Application isconnected]} {error "Unexpected existing application connection"}
    Application executable [file join [pwd] bin CarMaker.linux64]
    Application options "-screen -tstop $CMStationarySeconds -taccel 1"
    set CMStationaryOwnsApp 1
    Application start
    LoadTestRun LaneChange_ISO_Autoware
    SetSimTimeAcc 1
    set result [StartSim]
    if {$result ne ""} {error "StartSim refused: $result"}
    puts "Owned stationary Office simulation started"
} message]} {
    puts stderr "Stationary Office startup failed: $message"
    CMStationaryEnd
}
