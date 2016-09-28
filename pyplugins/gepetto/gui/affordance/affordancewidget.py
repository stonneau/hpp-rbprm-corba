from PythonQt import QtGui, Qt, QtCore
from gepetto.corbaserver import Client
from hpp.corbaserver.rbprm import Client as rbprmClient
from hpp.corbaserver import Client as basicClient
from hpp.corbaserver.affordance import Client as affClient

### This class represents one special tab of the new QDockWidget
class _RbprmPath (QtGui.QWidget):
    def __init__(self, parent, plugin):
        super(_RbprmPath, self).__init__ (parent)
        self.plugin = plugin
        vbox = QtGui.QVBoxLayout(self)
        robotLabel = QtGui.QLabel("Set up rbprm models:")
        vbox.addWidget(robotLabel)
        vbox.addWidget(self.addWidgetsInHBox([self.bindFunctionToButton("Load ROM robot",\
                self.Robot), self.bindFunctionToButton("Load environment", self.Environment)]))
        filterLabel = QtGui.QLabel("Add filters:")
        vbox.addWidget(filterLabel)
        self.ROMlist = QtGui.QListWidget()
        self.ROMlist.setSelectionMode(QtGui.QAbstractItemView.ExtendedSelection)
        filterLabel2 = QtGui.QLabel("Choose ROM:")
        self.ROMfilterList = QtGui.QListWidget()
        filterLabel3 = QtGui.QLabel("Existing filters:")
        vbox.addWidget(self.addWidgetsInHBox([filterLabel2,filterLabel3]))
        vbox.addWidget(self.addWidgetsInHBox([self.ROMlist, self.bindFunctionToButton("Add",\
                self.addFilter), self.ROMfilterList]))
        filterLabel = QtGui.QLabel("Add affordance filters:")
        vbox.addWidget(filterLabel)
        self.ROMaffList = QtGui.QListWidget()
        self.ROMaffList.setSelectionMode(QtGui.QAbstractItemView.ExtendedSelection)
        self.ROMaffFilterList = QtGui.QTreeWidget()
        filterLabel4 = QtGui.QLabel("Choose ROM")
        filterLabel5 = QtGui.QLabel("Existing affordance filters:")
        self.affTypeList = QtGui.QListWidget()
        self.affTypeList.setSelectionMode(QtGui.QAbstractItemView.ExtendedSelection)
        vbox.addWidget(self.addWidgetsInHBox([filterLabel4,filterLabel5]))
        vbox.addWidget(self.addWidgetsInHBox([self.ROMaffList, self.affTypeList,\
                self.bindFunctionToButton("Add",\
                self.addAffFilter), self.ROMaffFilterList]))
        vbox.addWidget(self.bindFunctionToButton("Panic!",self.addToViewer))
        self.update ()

    def update(self):
        self.ROMlist.clear()
        self.ROMaffList.clear()
        self.ROMaffFilterList.clear()
        ROMnames = self.plugin.rbprmClient.rbprm.getROMnames ()
        for name in ROMnames:
            self.ROMlist.addItem (name)
            self.ROMaffList.addItem (name)
            afffilters = self.plugin.rbprmClient.rbprm.getAffordanceFilter(name)
            if (len(afffilters) != 0):
                item = QtGui.QTreeWidgetItem()
                item.setText(0,name)
                for affi in afffilters:
                    child = QtGui.QTreeWidgetItem()
                    child.setText(0,affi)
                    item.addChild(child)
                self.ROMaffFilterList.addTopLevelItem(item)
                self.ROMaffFilterList.expandItem(item)
        self.ROMfilterList.clear()
        filters = self.plugin.rbprmClient.rbprm.getFilter ()
        for fi in filters:
            self.ROMfilterList.addItem(fi)
        self.affTypeList.clear()
        affs = self.getAffordanceConfigTypes ()
        for aff in affs:
            self.affTypeList.addItem(aff)

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

    def Robot (self):
        self.robot = "load"
        self.robotdialog = _RobotDialog()
        self.robotdialog.exec_()
        name = str(self.robotdialog.Envname.text)
        packageName = str(self.robotdialog.pkgName.text)
        urdfName =  str(self.robotdialog.urdfName.text)
        meshPackageName = str(self.robotdialog.mpkgName.text)
        urdfROMs = str(self.robotdialog.urdfromName.text)
        urdfROMs = urdfROMs.replace(" ", "")
        urdfNameRom = urdfROMs.split(',')
        rootJointType = str(self.robotdialog.rootJoint.currentText)
        urdfSuffix = str(self.robotdialog.urdfSuf.text)
        srdfSuffix = str(self.robotdialog.srdfSuf.text)
        self.loadModel(name, urdfName, urdfNameRom, rootJointType, \
                meshPackageName, packageName, urdfSuffix, srdfSuffix)
        self.update()

    def Environment (self):
        self.environment = "manual"
        self.envdialog = _EnvDialog()
        self.envdialog.exec_()
        name = str(self.envdialog.Envname.text)
        urdfName =  str(self.envdialog.urdfName.text)
        packageName = str(self.envdialog.pkgName.text)
        self.plugin.basicClient.obstacle.loadObstacleModel (packageName, urdfName, name)
        self.update()

    def loadModel (self, name, urdfName, urdfNameroms, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix):
            if(isinstance(urdfNameroms, list)):    
                for urdfNamerom in urdfNameroms:
                    self.plugin.rbprmClient.rbprm.loadRobotRomModel(urdfNamerom, rootJointType, packageName, urdfNamerom, urdfSuffix, srdfSuffix)
            else:
                self.plugin.rbprmClient.rbprm.loadRobotRomModel(urdfNameroms, rootJointType, packageName, urdfNameroms, urdfSuffix, srdfSuffix)
            self.plugin.rbprmClient.rbprm.loadRobotCompleteModel(name, rootJointType, packageName, urdfName, urdfSuffix, srdfSuffix)

    def addFilter (self):
        items = self.ROMlist.selectedItems()
        if (len(items) > 0):
            ROMnames = []
            for item in items:
                ROMnames.append(str(item.text()))
            self.plugin.rbprmClient.rbprm.setFilter(ROMnames)
        self.update()

    def addAffFilter (self):
        items = self.ROMaffList.selectedItems()
        affItems = self.affTypeList.selectedItems()
        if (len(items) > 0 and len (affItems) > 0):
            affNames = []
            for aff in affItems:
                affNames.append(str(aff.text()))
            for item in items:
                self.plugin.rbprmClient.rbprm.setAffordanceFilter(str(item.text()), affNames)
        self.update()

    def getAffordanceConfigTypes (self):
        return self.plugin.affClient.affordance.getAffordanceConfigTypes ()

    def addToViewer (self):
        self.plugin.client.gui.addUrdfObjects("hyq","/local/anna/devel/install/share/hyq_description",\
                "/local/anna/devel/install/share/hyq_description",True)

class _EnvDialog(QtGui.QDialog):
    def __init__(self, parent=None):
        super(_EnvDialog, self).__init__(parent)
        self.grid = QtGui.QGridLayout()
        self.grid.setSpacing(10)
        self.setGeometry(440,418, 450, 450)
        self.setWindowTitle('Input dialog')
        # Create widgets
        label = QtGui.QLabel("Choose predefined model or provide custom description.")
        self.model = QtGui.QComboBox()
        self.model.editable = False
        text1 = QtGui.QLabel('Choose predefined model')
        text2 = QtGui.QLabel('Model name')
        self.Envname = QtGui.QLineEdit("planning")
        self.button = QtGui.QPushButton("Details")
        self.button.setCheckable(True)

        # add widgets to grid
        self.grid.addWidget (label, 0,0)
        self.grid.addWidget (text1, 1,0)
        self.grid.addWidget (self.model, 1,1)
        self.grid.addWidget (text2,2,0,2,1)
        self.grid.addWidget (self.Envname, 2,1,2,1)
        self.grid.addWidget (self.button, 4,1,1,1)
        # create grid 2 and its widgets
        self.groupbox = QtGui.QGroupBox()
        self.grid2 = QtGui.QGridLayout(self.groupbox)
        self.grid2.setSpacing(10)
        envText3 = QtGui.QLabel('Package name')
        self.pkgName = QtGui.QLineEdit("hpp-rbprm-corba")
        envText4 = QtGui.QLabel('Mesh package name')
        self.mpkgName = QtGui.QLineEdit()
        envText5 = QtGui.QLabel('URDF filename')
        self.urdfName = QtGui.QLineEdit("darpa")
        envText7 = QtGui.QLabel('URDF suffix')
        self.urdfSuf = QtGui.QLineEdit()
        envText8 = QtGui.QLabel('SRDF suffix')
        self.srdfSuf = QtGui.QLineEdit()

        # add widgets to grid2
        self.grid2.addWidget (label, 0,0)    
        self.grid2.addWidget (envText3, 1,0)
        self.grid2.addWidget (self.pkgName, 1,2)     
        self.grid2.addWidget (envText4, 2,0)
        self.grid2.addWidget (self.mpkgName, 2,2)     
        self.grid2.addWidget (envText5, 3,0)
        self.grid2.addWidget (self.urdfName, 3,2)      
        self.grid2.addWidget (envText7, 5,0)
        self.grid2.addWidget (self.urdfSuf, 5,2)
        self.grid2.addWidget (envText8, 6,0)
        self.grid2.addWidget (self.srdfSuf, 6,2)
       
        self.OKbutton = QtGui.QPushButton("Load")
        self.CANCELbutton = QtGui.QPushButton("Cancel")
        self.grid.addWidget (self.groupbox,5,0,3,2)
        self.groupbox.setHidden(True)
        self.grid.addWidget (self.CANCELbutton, 8,0,1,1)
        self.grid.addWidget (self.OKbutton, 8,1,1,1)

        # Set dialog layout
        self.setLayout(self.grid)
        # Add button signal to showDetails slot
        self.button.clicked.connect(lambda: self.hideWidget(self.groupbox, self.button))
        self.OKbutton.clicked.connect(self.load)
        self.CANCELbutton.clicked.connect(self.cancel)
    # methods
    def load (self):
            self.accept()
    def cancel (self):
            self.reject()
    def hideWidget (self,widget, button):
        if button.isChecked():
            widget.setHidden(False)
        else:
            widget.setHidden(True)
        
class _RobotDialog(_EnvDialog):
    def __init__(self, parent=None):
        super(_RobotDialog, self).__init__(parent)
        #self.grid2.removeWidget(self.envText6)
        #self.grid2.removeWidget(self.urdfromName)
        text = QtGui.QLabel('Root-joint type')
        self.rootJoint = QtGui.QComboBox()
        self.rootJoint.addItem("freeflyer")
        self.rootJoint.addItem("planar")
        self.rootJoint.addItem("anchor")
        self.grid.addWidget(text,3,0,1,1)
        self.grid.addWidget(self.rootJoint,3,1,2,1)
        self.envText6 = QtGui.QLabel('URDF ROM names, separated by commas')
        self.urdfromName = QtGui.QLineEdit("hyq_rhleg_rom, hyq_lfleg_rom, hyq_rfleg_rom, hyq_lhleg_rom")
        self.grid2.addWidget (self.envText6, 4,0)
        self.grid2.addWidget (self.urdfromName, 4,2)
        self.Envname.setText("hyq_trunk_large")
        self.urdfName.setText("hyq_trunk_large")
        self.pkgName.setText("hpp-rbprm-corba")
        self.mpkgName.setText("hpp-rbprm-corba")


### This class represents one special tab of the new QDockWidget
class _AffCreator (QtGui.QWidget):
    def __init__(self, parent, plugin):
        super(_AffCreator, self).__init__ (parent)
        self.plugin = plugin
        vbox1 = QtGui.QVBoxLayout(self)
        # vbox1.setGeometry(QtCore.QRect(1000,1000,1000,1000))
        # vbox1.setSpacing(20)
        gridW = QtGui.QWidget()#(self)
        grid = QtGui.QGridLayout(gridW)
        
        #hbox1.setSpacing (0) #no effect
        #box.setMargin(0)
        #self.affMargin.setAlignment(Qt.Qt.AlignLeft)
        
        # Affordance Types
        self.affordanceTypes = QtGui.QComboBox(self)
        self.affordanceTypes.editable = False
        self.affordanceTypes.setMaximumWidth(100)
        affTypeLabel = QtGui.QLabel("Affordance Type:")
        affTypeLabel.setMaximumWidth(200)

        grid.addWidget(self.addWidgetsInHBox( [affTypeLabel,
            self.affordanceTypes]), 0,0)

        # Affordance Creation by Name
        self.affMargin = QtGui.QDoubleSpinBox() 
        self.affMargin.setValue (0.03)
        self.affMargin.setMaximumWidth(100)
        marginLabel = QtGui.QLabel("Normal Margin:")
        marginLabel.setMaximumWidth(200)
        grid.addWidget(self.addWidgetsInHBox([marginLabel, self.affMargin]), 1,0)
        self.affNTriMargin = QtGui.QDoubleSpinBox()
        self.affNTriMargin.setValue (0.03)
        self.affNTriMargin.setMaximumWidth(100)
        nTMarginLabel = QtGui.QLabel("Neighbour-Triangle Margin:")
        nTMarginLabel.setMaximumWidth(200)
        grid.addWidget(self.addWidgetsInHBox([nTMarginLabel, self.affNTriMargin]), 0,1)
        self.affMinArea = QtGui.QDoubleSpinBox()
        self.affMinArea.setValue (0.05)
        self.affMinArea.setMaximumWidth(100)
        areaLabel = QtGui.QLabel("Minimum Area:")
        areaLabel.setMaximumWidth(200)
        grid.addWidget(self.addWidgetsInHBox([areaLabel, self.affMinArea]), 1,1)
        
        vbox1.addWidget(gridW)

        # Add Affordance Configuration
        vbox1.addWidget(self.bindFunctionToButton("Edit", self.setAffConfig))
        # Affordance Analysis Button
        self.affAnalysisObjects = QtGui.QComboBox(self)
        self.affAnalysisObjects.editable = False
        self.affAnalysisOptions = QtGui.QComboBox(self)
        self.affAnalysisOptions.editable = False
       
        self.colour = [0.7450980392156863, 1.0, 0.3333333333333333, 1]
        self.affColour = QtGui.QAction('Font bg Color', self)
        self.affColour.triggered.connect(self.colour_picker)

        vbox1.addWidget(self.addWidgetsInHBox( [
            QtGui.QLabel("Object:"), self.affAnalysisObjects,
            QtGui.QLabel("Affordance type:"), self.affAnalysisOptions]))

        vbox1.addWidget(self.bindFunctionToButton("Choose Colour", self.colour_picker))
        vbox1.addWidget(self.bindFunctionToButton("Find Affordances", self.affordanceAnalysis))

        self.deleteObjects = QtGui.QComboBox(self)
        self.deleteObjects.editable = False
        self.deleteAffs = QtGui.QComboBox(self)
        self.deleteAffs.editable = False
        vbox1.addWidget(self.addWidgetsInHBox( [
            QtGui.QLabel("Object:"), self.deleteObjects,
            QtGui.QLabel("Affordance type:"), self.deleteAffs]))
        vbox1.addWidget(self.bindFunctionToButton("Delete Affordances", self.deleteAffordancesByType))

        self.update()

    def update(self):
        self.affordanceTypes.clear()
        self.affAnalysisOptions.clear()
        affordances = self.getAffordanceConfigTypes()
        for aff in affordances:
            self.affordanceTypes.addItem (aff)
            self.affAnalysisOptions.addItem (aff)
        self.affAnalysisOptions.addItem ("All types")
        self.affAnalysisObjects.clear ()
        self.deleteObjects.clear ()
        objects = self.plugin.basicClient.obstacle.getObstacleNames(False,True)
        for obj in objects:
            self.affAnalysisObjects.addItem (obj)
            self.deleteObjects.addItem (obj)
        self.affAnalysisObjects.addItem ("All objects")
        self.deleteObjects.addItem ("All objects")
        self.deleteAffs.clear ()
        affordances = self.getAffordanceTypes()
        for aff in affordances:
            self.deleteAffs.addItem (aff)
        self.deleteAffs.addItem ("All types")

    def addWidgetsInHBox(self, widgets):
        nameParentW = QtGui.QWidget(self)
        hboxName = QtGui.QHBoxLayout(nameParentW)
        for w in widgets:
            hboxName.addWidget(w)
        return nameParentW

    def addWidgetsInVBox(self, widgets):
        nameParentW = QtGui.QWidget(self)
        vboxName = QtGui.QVBoxLayout(nameParentW)
        for w in widgets:
            vboxName.addWidget(w)
        return nameParentW

    def bindFunctionToButton (self, buttonLabel, func):
        button = QtGui.QPushButton(self)
        button.text = buttonLabel
        button.connect ('clicked()', func)
        return button

    def colour_picker (self):
        colour = QtGui.QColorDialog.getColor()
        self.colour = [colour.redF(), colour.greenF(), colour.blueF(), colour.alphaF()]
        print (self.colour)

    def addMesh (self):
        filename = QtGui.QFileDialog.getOpenFileName (self, "Choose a mesh")
        self.plugin.client.gui.addMesh(str(self.nodeName.text), str(filename))
        self.refreshBodyTree()

    def addBox (self):
        self.plugin.client.gui.addBox(str(self.nodeName.text), 1, 1, 1, self.colour)
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
        affType = str(self.affAnalysisOptions.currentText)
        if objectname == "All objects":
            objectname = ""
            self.plugin.affClient.affordance.analyseAll ()
        else:
            self.plugin.affClient.affordance.analyseObject (objectname)
        if affType == "All types":
            affordances = self.getAffordanceTypes ()
            for aff in affordances:
                 self.visualiseAffordances(aff, self.colour, objectname)
        else:
            self.visualiseAffordances(affType, self.colour, objectname)
        self.update ()

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
        groupNodes = self.plugin.client.gui.getGroupNodeList("hpp-gui")
        self.plugin.client.gui.addToGroup (str (affType), "hpp-gui")
        # By default, oldest node is displayed in front. Removing and re-adding
        # object from scene assure that the new triangles are displayed on top     
        for groupNode in groupNodes :
            self.plugin.client.gui.removeFromGroup(groupNode, "hpp-gui")
            self.plugin.client.gui.addToGroup(groupNode, "hpp-gui")
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
          groupNodes = self.plugin.client.gui.getGroupNodeList('hpp-gui')
          self.plugin.client.gui.addToGroup (str (affType), 'hpp-gui')
          # By default, oldest node is displayed in front. Removing and re-adding i
          # object from scene assure that the new triangles are displayed on top     
          for groupNode in groupNodes :
              self.plugin.client.gui.removeFromGroup(groupNode,'hpp-gui')
              self.plugin.client.gui.addToGroup(groupNode,'hpp-gui')
        return

    def deleteNode (self, nodeName, all):
        return self.plugin.client.gui.deleteNode (nodeName, all)

    def getAffordanceTypes (self):
        return self.plugin.affClient.affordance.getAffordanceTypes ()

    def getAffordanceConfigTypes (self):
        return self.plugin.affClient.affordance.getAffordanceConfigTypes ()

    def getAffRefObstacles (self, affType):
        return self.plugin.affClient.affordance.getAffRefObstacles (affType)

    def deleteAffordances (self, obstacleName=""):
        self.deleteAffordancesFromViewer (obstacleName)
        return self.plugin.affClient.affordance.deleteAffordances (obstacleName)

    def deleteAffordancesFromViewer (self, obstacleName=""):
        affs = self.getAffordanceTypes ()
        if obstacleName == "":
            for aff in affs:
                self.deleteNode (aff, True)
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
                     self.deleteNode (node, True)
               count += 1
        return

    def deleteAffordancesByType (self):
        objectname = str(self.deleteObjects.currentText)
        affType = str(self.deleteAffs.currentText)
        if objectname == "All objects":
            objectname = ""
        if affType == "All types":
            affType == ""
            self.plugin.affClient.affordance.deleteAffordances(objectname)
        else:
            self.deleteAffordancesByTypeFromViewer (affType, objectname)
            self.plugin.affClient.affordance.deleteAffordancesByType (affType, objectname)
        self.update ()
        return

    def deleteAffordancesByTypeFromViewer (self, affordanceType, obstacleName=""):
        if obstacleName == "":
          self.deleteNode (affordanceType, True)
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
        self.tabWidget = QtGui.QTabWidget() #(self)
        self.setWidget (self.tabWidget)
        self.rbprmPath = _RbprmPath (self,self)
        self.tabWidget.addTab (self.rbprmPath, "Rbprm Path")
        self.affCreator = _AffCreator(self, self)
        self.tabWidget.addTab (self.affCreator, "Affordance Creator")
        layout1 = QtGui.QHBoxLayout()
        layout1.addWidget(QtGui.QLabel("Testing Tab"))
        widget = QtGui.QWidget()
        widget.setLayout(layout1)
        self.tabWidget.addTab (widget, "Affordance Creator Tab2")
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
        self.affCreator.update()

    def selected(self, name, posInWorldFrame):
        QtGui.QMessageBox.information(self, "Selected object", name + " " + str(posInWorldFrame))
