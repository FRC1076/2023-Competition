
from datetime import datetime
import wpilib

class Logger:
    theLogger = None 

    def __init__(self, dir=None):
        self.dataLog = wpilib.DataLogManager
        self.dataLog.start(dir)

    def log(self, *dataToLog):
        dataAsStrings = map(lambda x: str(x), dataToLog)
        message = ', '.join(dataAsStrings)

        timestamp = datetime.now().strftime("%m/%d/%Y-%m-%d-%H:%M:%S")
        logEntry = 'LOG: (' + timestamp + ') ' + message
        self.dataLog.log(logEntry)

    # singleton pattern!
    def getLogger(dir=''):
        if not Logger.theLogger:
            Logger.theLogger = Logger(dir)
        return Logger.theLogger