#!/usr/bin/env python

"""
Simplified script to build and start the SITL ardupilot simulation for Planck Aerosystems simulation purposes.
Inspired from Tools/autotest/sim_vehicle.py

Does not span a Mavproxy GCS but waits for a GCS TCP connection on port 5760

"""
import os
import os.path
import signal
import subprocess
import sys
import time

abspath = os.path.abspath(__file__)
dname = os.path.dirname(abspath)
root_dir = dname.replace('/planckAero', '')

def progress_cmd(what, cmd):
    """Print cmd in a way a user could cut-and-paste to get the same effect"""
    progress(what)
    shell_text = "%s" % (" ".join(['"%s"' % x for x in cmd]))
    progress(shell_text)

def run_cmd_blocking(what, cmd, quiet=False, check=False, **kw):
    if not quiet:
        progress_cmd(what, cmd)
    p = subprocess.Popen(cmd, **kw)
    ret = os.waitpid(p.pid, 0)
    _, sts = ret
    if check and sts != 0:
        progress("(%s) exited with code %d" % (what,sts,))
        sys.exit(1)
    return ret

def progress(text):
    """Display sim_vehicle progress text"""
    print("SIM_VEHICLE: " + text)

def do_build_waf(frame_options):
    """Build sitl using waf"""
    progress("WAF build")

    # Go in the root folder since waf need to be called from build folder
    old_dir = os.getcwd()
    os.chdir(root_dir)

    # Waf path
    waf_light = os.path.join("modules/waf/waf-light")

    # Configure waf for SITL board
    cmd_configure = [waf_light, "configure", "--board", "sitl"]
    run_cmd_blocking("Configure waf", cmd_configure, check=True)

    # Start the build
    cmd_build = [waf_light, "build", "--target", frame_options["waf_target"]]
    _, sts = run_cmd_blocking("Building", cmd_build)

    # If build failed
    if sts != 0:
        # Try a clean build
        progress("Build failed; cleaning and rebuilding")
        run_cmd_blocking("Building clean", [waf_light, "clean"])
        _, sts = run_cmd_blocking("Building", cmd_build)

        #Still fails
        if sts != 0:
            progress("Build failed")
            sys.exit(1)

    os.chdir(old_dir)

#Starting script
progress("Start Ardupilot SITL")

#Defining variables:
binary_basedir = os.path.join(root_dir, "build/sitl");

#Defining frame
frame_options = {
        "model": "+",
        "waf_target": "bin/arducopter-quad",
        "default_params_filename": "default_params/copter.parm"};

#Define location at start-up: Need to be set accordingly with the initial platform location of the planck_ctrl simulation
location = "32.715736,-117.161087,0,0";

#Building Arducopter
progress("Building Sitl: ")
do_build_waf(frame_options)

#Set option for starting Arducopter SITL
vehicle_binary = os.path.join(binary_basedir, frame_options["waf_target"])
cmd = []
cmd.append(vehicle_binary)
cmd.append("-S")
cmd.append("-I0")
cmd.extend(["--home", location])
cmd.extend(["--model", "x"])
cmd.extend(["--speedup", "1"])

#Using default parameters
param_path = os.path.join(root_dir, "Tools/autotest", frame_options["default_params_filename"])
progress("Using defaults from (%s)" % (param_path,))
cmd.extend(["--defaults", param_path])

#Starting vehicle
progress("Starting vehicle")
local_mp_modules_dir = os.path.abspath(
    os.path.join(__file__, '..', '..', 'mavproxy_modules'))
env = dict(os.environ)
env['PYTHONPATH'] = local_mp_modules_dir + os.pathsep + env.get('PYTHONPATH', '')

run_cmd_blocking("Start Vehicle", cmd, env=env)
progress("Exiting vehicle")
sys.exit(0)
