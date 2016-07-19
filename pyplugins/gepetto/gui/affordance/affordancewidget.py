from PythonQt import QtGui, Qt
from gepetto.corbaserver import Client
from hpp.corbaserver.rbprm import Client as rbprmClient
from hpp.corbaserver import Client as basicClient
from hpp.corbaserver.affordance import Client as affClient

### This class represents one special tab of the new QDockWidget
class _AffCreator (QtGui.QWidget):
    def __init__(self, parent, plugin):
        super(_AffCreator, self).__init__ (parent)
        self.plugin = plugin
        box = QtGui.QVBoxLayout(self)

        # Affordance Types
        self.affordanceTypes = QtGui.QComboBox(self)
        self.affordanceTypes.editable = False
        box.addWidget(self.addWidgetsInHBox( [QtGui.QLabel("            Affordance Type:"),
            self.affordanceTypes]))

        # Affordance Creation by Name
        self.affMargin = QtGui.QDoubleSpinBox() 
        self.affMargin.setValue (0.03)
        box.addWidget(self.addWidgetsInHBox([QtGui.QLabel("               Normal Margin:"), self.affMargin]))
        self.affNTriMargin = QtGui.QDoubleSpinBox()
        self.affNTriMargin.setValue (0.03)
        box.addWidget(self.addWidgetsInHBox([QtGui.QLabel("Neighbouring Triangle Margin:"), self.affNTriMargin]))
        self.affMinArea = QtGui.QDoubleSpinBox()
        self.affMinArea.setValue (0.05)
        box.addWidget(self.addWidgetsInHBox([QtGui.QLabel("                Minimum Area:"), self.affMinArea]))
        

        # Name line edit
        #self.nodeName = QtGui.QLineEdit("nodeName")
        #box.addWidget(self.addWidgetsInHBox([QtGui.QLabel("Node name:"), self.nodeName]))

        # Create group
        #box.addWidget(self.bindFunctionToButton("Create group", self.createGroup))

        # Add Affordance Configuration
        box.addWidget(self.bindFunctionToButton("Edit Affordance Configuration", self.setAffConfig))

    #    # Add to group
    #    self.groupNodes = QtGui.QComboBox(self)
    #    self.groupNodes.editable = False
    #    box.addWidget(self.addWidgetsInHBox( [
    #        self.groupNodes,
    #        self.bindFunctionToButton("Add to group", self.addToGroup)
    #            ]))

        # Affordance Analysis Button
        self.affAnalysisObjects = QtGui.QComboBox(self)
        self.affAnalysisObjects.editable = False
        self.affAnalysisOptions = QtGui.QComboBox(self)
        self.affAnalysisOptions.editable = False
        box.addWidget(self.addWidgetsInHBox( [
            QtGui.QLabel("Choose object:"), self.affAnalysisObjects,
            QtGui.QLabel("Choose affordance type:"), self.affAnalysisOptions]))

        box.addWidget(self.bindFunctionToButton("Find Affordances", self.affordanceAnalysis))
        # Add mesh
        #box.addWidget(self.bindFunctionToButton("Add mesh", self.addMesh))

        # Add box
        #box.addWidget(self.bindFunctionToButton("Add box", self.addBox))

        self.update()

    def update(self):
    #    self.groupNodes.clear()
    #    for n in self.plugin.client.gui.getSceneList():
    #        self.groupNodes.addItem (n)
        self.affordanceTypes.clear()
        self.affAnalysisOptions.clear()
        affordances = self.getAffordanceTypes()
        for aff in affordances:
            self.affordanceTypes.addItem (aff)
            self.affAnalysisOptions.addItem (aff)
        self.affAnalysisOptions.addItem("All Types")
        self.affAnalysisObjects.clear()
        objects = self.plugin.basicClient.obstacle.getObstacleNames(True,False)
        for obj in objects:
            self.affAnalysisObjects.addItem (obj)
        self.affAnalysisObjects.addItem ("all objects")

    def addWidgetsInHBox(self, widgets):
        nameParentW = QtGui.QWidget(self)
        hboxName = QtGui.QHBoxLayout(nameParentW)
        for w in widgets:
            hboxName.addWidget(w)
        return nameParentW

    def bindFunctionToButton (self, buttonLabel, func):
        button = QtGui.QPushButton(self)
        button.text = buttonLabel
        button.connect ('clicked()', func)
        return button

    def addMesh (self):
        filename = QtGui.QFileDialog.getOpenFileName (self, "Choose a mesh")
        self.plugin.client.gui.addMesh(str(self.nodeName.text), str(filename))
        self.refreshBodyTree()

    def addBox (self):
        self.plugin.client.gui.addBox(str(self.nodeName.text), 1, 1, 1, [1, 0, 0, 1])
        self.refreshBodyTree()

    def createGroup (self):
        self.plugin.client.gui.createGroup(str(self.nodeName.text))
        self.groupNodes.addItem(self.nodeName.text)
        self.refreshBodyTree()

    def addToGroup (self):
        self.plugin.client.gui.addToGroup(str(self.nodeName.text), str(self.groupNodes.currentText))
        self.refreshBodyTree()

    def setAffConfig (self):
        config = [self.affMargin.value, self.affNTriMargin.value, self.affMinArea.value]
        self.plugin.affClient.affordance.setAffordanceConfig (str(self.affordanceTypes.currentText), config)

    def affordanceAnalysis (self):
        objectname = str(self.affAnalysisObjects.currentText)
        if objectname == "all objects":
            objectname = ""
            self.plugin.affClient.affordance.analyseAll ()
        else:
            self.plugin.affClient.affordance.analyseObject (str(self.affAnalysisObjects.currentText))
        self.visualiseAffordances("Support", [0.25, 0.5, 0.5, 1], objectname)

    def getAffordancePoints (self, affordanceType):
        return self.plugin.affClient.affordance.getAffordancePoints (affordanceType)

    def visualiseAllAffordances (self, affType, colour):
        if len(colour) < 4 : # if the colour is only rgb we suppose alpha = 1 
          colour = colour + [1]
        self.deleteNode (str (affType), True)
        objs = self.getAffordancePoints (affType)
        refs = self.getAffRefObstacles (affType)
        self.plugin.client.gui.createGroup (str (affType))
        # TODO: add to groupNodes / refresh Body tree?
        for aff in objs:
          count = 0
          for tri in aff:
            self.plugin.client.gui.addTriangleFace (str (affType) + '-' + \
                 str (refs[objs.index (aff)]) + '.' + \
                 str (objs.index (aff)) + '.' + str(count), \
                 tri[0], tri[1], tri[2], [colour[0], colour[1], colour[2], colour[3]])
            self.plugin.client.gui.addToGroup (str (affType) + '-' + \
                 str (refs[objs.index (aff)]) + '.' + \
                 str (objs.index (aff)) + '.' + str(count), str (affType))
            count += 1
        scenes = self.plugin.client.gui.getSceneList() #TODO: add check to see whether scenes is empty
        print (len(scenes))
        print (len(scenes[0]))
        groupNodes = self.plugin.client.gui.getGroupNodeList(scenes[0])
        self.plugin.client.gui.addToGroup (str (affType), scenes[0])
        print ("Still alive")
        # By default, oldest node is displayed in front. Removing and re-adding
        # object from scene assure that the new triangles are displayed on top     
        for groupNode in groupNodes :
            self.plugin.client.gui.removeFromGroup(groupNode, scenes[0])
            self.plugin.client.gui.addToGroup(groupNode, scenes[0])
        return

    def visualiseAffordances (self, affType, colour, obstacleName=""):
        if len(colour) < 4 : # if the colour is only rgb we suppose alpha = 1 
          colour = colour + [1]
        if obstacleName == "":
          return self.visualiseAllAffordances (affType, colour)
        else:
          self.deleteAffordancesByTypeFromViewer (affType, obstacleName)
          nodes = self.plugin.client.gui.getNodeList ()
          if affType not in nodes: self.plugin.client.gui.createGroup (str (affType))
          objs = self.getAffordancePoints (affType)
          refs = self.getAffRefObstacles (affType)
          for aff in objs:
            if refs[objs.index (aff)] == obstacleName:
              count = 0
              for tri in aff:
                name = str (affType) + '-' + \
                str (refs[objs.index (aff)]) + '.' + \
         str (objs.index (aff)) + '.' + str(count)
                self.plugin.client.gui.addTriangleFace (name, \
                    tri[0], tri[1], tri[2], [colour[0], colour[1], colour[2], colour[3]])
                self.plugin.client.gui.addToGroup (name, str (affType))
                count += 1
          scenes = self.plugin.client.gui.getSceneList() #TODO: add check to see whether scenes is empty
          groupNodes = self.plugin.client.gui.getGroupNodeList(scenes[0])
          self.plugin.client.gui.addToGroup (str (affType), scenes[0])
          # By default, oldest node is displayed in front. Removing and re-adding i
          # object from scene assure that the new triangles are displayed on top     
          for groupNode in groupNodes :
              self.plugin.client.gui.removeFromGroup(groupNode,scenes[0])
              self.plugin.client.gui.addToGroup(groupNode,scenes[0])
        return

    def deleteNode (self, nodeName, all):
        return self.plugin.client.gui.deleteNode (nodeName, all)

    def getAffordanceTypes (self):
        return self.plugin.affClient.affordance.getAffordanceTypes ()

    def getAffRefObstacles (self, affType):
        return self.plugin.affClient.affordance.getAffRefObstacles (affType)

    def deleteAffordances (self, obstacleName=""):
        self.deleteAffordancesFromViewer (obstacleName)
        return self.plugin.affClient.affordance.deleteAffordances (obstacleName)

    def deleteAffordancesFromViewer (self, obstacleName=""):
        affs = self.getAffordanceTypes ()
        if obstacleName == "":
            for aff in affs:
                self.deleteNode (aff, True, Viewer)
        else:
           import re
           for aff in affs:
             refs = self.getAffRefObstacles (aff)
             count = 0
             while count < len(refs):
               if refs[count] == obstacleName:
                 toDelete = aff + '-' + refs[count]
                 nodes = self.plugin.client.gui.getGroupNodeList(aff)
                 for node in nodes:
                   splt = re.split ('\.', node)
                   if splt[0] == toDelete:
                     self.deleteNode (node, True, Viewer)
               count += 1
        return

    def deleteAffordancesByType (self, affordanceType, obstacleName=""):
        self.deleteAffordancesByTypeFromViewer (affordanceType, obstacleName)
        return self.plugin.affClient.affordance.deleteAffordancesByType(affordanceType, obstacleName)

    def deleteAffordancesByTypeFromViewer (self, affordanceType, obstacleName=""):
        if obstacleName == "":
          self.plugin.affClient.gui.deleteNode (affordanceType, True)
        else:
           import re
           affs = self.getAffordanceTypes ()
           for aff in affs:
             if aff == affordanceType:
               refs = self.getAffRefObstacles (aff)
               count = 0
               while count < len(refs):
                 if refs[count] == obstacleName:
                   toDelete = aff + '-' + refs[count]
                   nodes = self.plugin.client.gui.getNodeList()
                   for node in nodes:
                     splt = re.split ('\.', node)
                     if splt[0] == toDelete:
                       self.deleteNode (node, True)
                 count += 1
        return


    ## See gepetto::gui::MainWindow::requestRefresh for more information
    def refreshBodyTree(self):
        self.plugin.main.requestRefresh()

class Plugin(QtGui.QDockWidget):
    """
    This plugin interacts with PythonWidget C++ class.
    """
    def __init__ (self, mainWindow, flags = None):
        if flags is None:
            super(Plugin, self).__init__ ("Affordance plugin", mainWindow)
        else:
            super(Plugin, self).__init__ ("Affordance plugin", mainWindow, flags)
        self.client = Client ()
        self.rbprmClient = rbprmClient ()
        self.basicClient  = basicClient ()
        self.affClient = affClient ()
        # Initialize the widget
        self.tabWidget = QtGui.QTabWidget(self)
        self.setWidget (self.tabWidget)
        self.nodeCreator = _AffCreator(self, self)
        self.tabWidget.addTab (self.nodeCreator, "Affordance Creator")
        self.main = mainWindow
        mainWindow.connect('refresh()', self.refresh)

    ### If present, this function is called when a new OSG Widget is created.
    def osgWidget(self, osgWindow):
        osgWindow.connect('selected(QString,QVector3D)', self.selected)

    def resetConnection(self):
        self.client = Client()
        self.rbprmClient = rbprmClient ()
        self.basicClient = basicClient ()
        self.affClient = affClient ()

    def refresh(self):
        self.nodeCreator.update()

    def selected(self, name, posInWorldFrame):
        QtGui.QMessageBox.information(self, "Selected object", name + " " + str(posInWorldFrame))
