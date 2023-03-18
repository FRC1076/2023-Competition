
from networktables import NetworkTables

class Dashboard:

    theDashboard = None

    def __init__(self, testMode):
        self.tableName = 'SmartDashboard'
        self.server = 'roborio-1076-frc.local'
        if testMode:
            self.server = 'localhost'
        NetworkTables.initialize(server=self.server) # Necessary for vision to
        self.dashboard = NetworkTables.getTable('SmartDashboard')

    def getTable(self):
        return self.dashboard
    
    def getDashboard(testMode=False):
        if not Dashboard.theDashboard:
            Dashboard.theDashboard = Dashboard(testMode)
        return Dashboard.theDashboard.getTable()
