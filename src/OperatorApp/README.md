# OperatorApp

The OperatorApp serves as the interface between the drone and the human operator.  
It runs on the Operator's PC and displays information about the drone and send user commands to it.

## File List:
*(% represents a variable or value)*

The following files are divided as follows:

- %.ui
    - the files created through the Qt Designer App

- pyuic5.exe
    - the application for converting the .ui files to .py

- DroneControls%.py
    - the previous python files that represent the main application for the drone operations
    - labelled from 1 to n based on the time coverted and edited
    - **currently not in online project file**

- UserInterface.py
    - the main python file that runs the entire GUI

- Main.py
    - the python file for redirecting to the main application

- MainLogin.py
    - the python file for redirecting to the main application after going through the login page

- icons
    - folder containing icons for logo and widgets like wifi, link, etc.