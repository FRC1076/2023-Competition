
from networktables import NetworkTables
from robotconfig import dashboardConfig, MODULE_NAMES

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
    
    def putNumber(self, moduleName, key, value):
        if dashboardConfig[moduleName]:
            self.dashboard.putNumber(moduleName + '/' + key, value)

    def putBoolean(self, moduleName, key, value):
        if dashboardConfig[moduleName]:
            self.dashboard.putBoolean(moduleName + '/' + key, value)

    def putString(self, moduleName, key, value):
        if dashboardConfig[moduleName]:
            self.dashboard.putString(moduleName + '/' + key, value)

    def getNumber(self, moduleName, key, defaultValue = -1):
        return self.dashboard.getNumber(moduleName + '/' + key, defaultValue)

    def getBoolean(self, moduleName, key, defaultValue = False):
        return self.dashboard.getBoolean(moduleName + '/' + key, defaultValue)
    
    def getString(self, moduleName, key, defaultValue = ''):
        return self.dashboard.getBoolean(moduleName + '/' + key, defaultValue)
    
    def getDashboard(testMode=False):
        if not Dashboard.theDashboard:
            Dashboard.theDashboard = Dashboard(testMode)
        return Dashboard.theDashboard
