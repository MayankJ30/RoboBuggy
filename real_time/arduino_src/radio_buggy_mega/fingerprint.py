""" fingerprint.py: 
* Description: This is a python script to create a .h file defining the time 
*   of compilation and who compiled.  Also details whether files had been
*   modified since the last commit
*
*
*   By Sean Buckley
"""


import os
import datetime
import socket #Used for getting hostname
import subprocess

def main():
    print("Now creating fingerprint.h...\n")
    file = open("fingerprint.h", 'w')
    file.write("/* fingerprint.h: A compilation-time created file containing compilation time, \n")
    file.write("closest version of git, and other information of */\n\n")
    time = datetime.datetime.now()
    file.write("#define COMPDATE \"%02d:%02d:%04d\" //day:month:year\n" %(time.day, time.month, time.year))
    file.write("#define COMPTIME \"%02d:%02d:%02d\" //hour:minute:second\n" %(time.hour, time.minute, time.second))
    file.write("#define HOSTNAME \"%s\"\n" %socket.gethostname().split(".")[0])


    branch = str(subprocess.check_output(["git", "symbolic-ref", "--short", "HEAD"])).replace("\n", "")

    file.write("#define BRANCHNAME \"%s\"\n" %branch)

    result = str(subprocess.check_output(["git", "status"]))

    clean = False

    for line in result.rsplit("\n"):
        if "working directory clean" in line:
            clean = True


    hString = str(subprocess.check_output(["git", "log", "-1", "--pretty=format:'%h'"])).rstrip().replace("'", "")

    file.write("#define HEXCOMMITHASH 0x%s \n" %(hString))
    file.write("#define STRCOMMITHASH \"%s\"\n" %(hString))
    file.write("#define CLEANSTATUS %s\n" %str(clean).lower())


    file.close()
    print("File created.\n")

main()