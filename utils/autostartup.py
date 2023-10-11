import pygetwindow
import os
import time
import webbrowser
from pathlib import Path
import psutil
import subprocess
import requests
from threading import Thread
import ctypes

from wpilib import RobotBase

# Kill previous dashboard processes

old_dashboard_processes = [p for p in psutil.process_iter() if
                           "pynetworktables2js" in p.name() or "chrome.exe" in p.name()]

for p in old_dashboard_processes:
    try:
        p.kill()
        print("Killed", p)
    except psutil.NoSuchProcess:
        print("Could not kill", p)

# DriverStation

os.startfile(r"C:\Program Files (x86)\FRC Driver Station\DriverStation.exe")

DriverStation = None
while not DriverStation:
    try:
        DriverStation = pygetwindow.getWindowsWithTitle("FRC Driver Station")[0]
    except IndexError:
        print('FRC Driver Station not detected')
        time.sleep(1)

DriverStation.moveTo(-8, -8)
time.sleep(3)
DriverStation.maximize()

# Dashboard

dashboard_path = str(Path(os.getcwd() + r"\.dashboard"))
subprocess.Popen("pynetworktables2js.exe --team 5528", shell=True, cwd=dashboard_path)
# subprocess.Popen("pynetworktables2js.exe --robot 127.0.0.1", shell=True, cwd=dashboard_path)

dashboard_url = "http://localhost:8888"

while True:
    try:
        result = requests.get(dashboard_url)
        if result.ok:
            break
    except requests.exceptions.ConnectionError:
        print("Dashboard not started")

# Chrome

# chrome_path = '"C:/Program Files/Google/Chrome/Application/chrome.exe"'
chrome_path = "C:/Program Files/Google/Chrome/Application/chrome.exe"
chrome_path += ' --profile-directory="Default" --app=%s'
chrome = webbrowser.get(chrome_path)
Thread(target=lambda: chrome.open(dashboard_url)).start()
time.sleep(1)

chrome_name = None

while chrome_name is None:
    try:
        chrome_name = [name for name in pygetwindow.getAllTitles() if "Fancy Web Dashboard" in name][0]
    except IndexError:
        print("Chrome window not found")
        time.sleep(1)

Browser = None

while Browser is None:
    try:
        Browser = pygetwindow.getWindowsWithTitle(chrome_name)[0]
        print(Browser)
    except IndexError:
        print('Chrome not detected')
        time.sleep(1)

print("Resizing Chrome")
time.sleep(3)
Browser.moveTo(-8, DriverStation.bottom - 8)
Browser.resizeTo(DriverStation.size.width, ctypes.windll.user32.GetSystemMetrics(1) - DriverStation.size.height + 16)

DriverStation.show()
Browser.show()
