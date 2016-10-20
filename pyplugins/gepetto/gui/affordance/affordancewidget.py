from PythonQt import QtGui, Qt, QtCore
from gepetto.corbaserver import Client
from hpp.corbaserver.rbprm import Client as rbprmClient
from hpp.corbaserver import Client as basicClient
from hpp.corbaserver.affordance import Client as affClient
from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from hpp.gepetto import Viewer
import math
import time
from threading import Thread

def magnitude(v):
    return math.sqrt(sum(v[i]*v[i] for i in range(len(v))))

def normalise(v):
    vmag = magnitude(v)
    if vmag == 0.0:
        vmag = 1.0
    return [ v[i]/vmag  for i in range(len(v)) ]

def checkBounds (bounds):
    if (len(bounds)%2 != 0):
        print ("Bounds vector has wrong length!")
    else:
        for i in range(len(bounds)/2):
            if bounds[i*2] > bounds[i*2+1]:
                return False
        return True

def equal (v1, v2):
    res = True
    if len(v1) == len(v2):
        for i in range (len(v1)):
            if v1[i] != v2[i]:
                res = False
                break
    else:
        res = False
    return res

### This class represents one special tab of the new QDockWidget
class _RbprmPath (QtGui.QWidget):
    def __init__(self, parent, plugin):
        super(_RbprmPath, self).__init__ (parent)
        self.plugin = plugin
        self.plugin.chooseWithMouse = False
        self.initConfigSet = False
        self.plugin.SO3bounds = [1,0,1,0,1,0]
        self.q_init = [0, 0, 0, 1, 0, 0, 0]
        self.q_goal = [0, 0, 0, 1, 0, 0, 0]
        self.lastObject = ""
        self.lastAff = ""
        self.analysed = False
        vbox = QtGui.QVBoxLayout(self)
        vbox.addWidget(self.addWidgetsInHBox([self.bindFunctionToButton("Load ROM robot",\
                self.Robot), self.bindFunctionToButton("Load environment", self.Environment)]))
        filterLabel = QtGui.QLabel("Add filters:")
        vbox.addWidget(filterLabel)
        self.groupbox2 = QtGui.QGroupBox()
        self.vbox2 = QtGui.QVBoxLayout(self.groupbox2)
        self.vbox2.addWidget(self.bindFunctionToButton("Add",self.addFilters))
        self.vbox2.addWidget(self.bindFunctionToButton("Clear",self.clearFilters))
        self.ROMaffList = QtGui.QListWidget()
        self.ROMaffList.setSelectionMode(QtGui.QAbstractItemView.ExtendedSelection)
        self.ROMaffFilterList = QtGui.QTreeWidget()
        filterLabel4 = QtGui.QLabel("Choose ROM")
        filterLabel5 = QtGui.QLabel("Existing affordance filters:")
        self.affTypeList = QtGui.QListWidget()
        self.affTypeList.setSelectionMode(QtGui.QAbstractItemView.ExtendedSelection)
      #  vbox.addWidget(self.addWidgetsInHBox([filterLabel4,filterLabel5]))
        vbox.addWidget(self.addWidgetsInHBox([self.ROMaffList, self.affTypeList,\
                self.groupbox2, self.ROMaffFilterList]))
        #----------------------- affrodance:
        self.affGroup = QtGui.QGroupBox()
        self.affGrid = QtGui.QGridLayout(self.affGroup)
        self.affGrid.setSpacing(10)
        # Affordance Analysis Button
        self.affAnalysisObjects = QtGui.QComboBox(self)
        self.affAnalysisObjects.editable = False
        self.affAnalysisOptions = QtGui.QComboBox(self)
        self.affAnalysisOptions.editable = False
        self.colourButton = self.bindFunctionToButton("Colour", self.colour_picker)
        qcolour = QtGui.QColor ("darkCyan");
        self.colour = [qcolour.redF(), qcolour.greenF(), qcolour.blueF(), qcolour.alphaF()]
        self.colourBox = QtGui.QGroupBox()
        self.colourBox.setStyleSheet("background-color:" + str(qcolour.name()))
        self.affGrid.addWidget(QtGui.QLabel("Affordance creator:"), 0,0)
    #    self.affGrid.addWidget(QtGui.QLabel("Object:"), 1,0)
        self.affGrid.addWidget(self.affAnalysisObjects,1,0)
     #   self.affGrid.addWidget(QtGui.QLabel("Affordance type:"),1,2)
        self.affGrid.addWidget(self.affAnalysisOptions, 1,1)
        self.affGrid.addWidget(self.colourButton, 1,2)
        self.affGrid.addWidget(self.colourBox, 1,3,2,1)
        self.affGrid.addWidget(self.bindFunctionToButton("Settings", self.Affordance), 2,0)
        self.affGrid.addWidget(self.bindFunctionToButton("Find", self.affordanceAnalysis), 2,1)
        self.affGrid.addWidget(self.bindFunctionToButton("Delete", self.deleteAffordancesByType), 2,2)
        vbox.addWidget (self.affGroup) 

        #---------------------------joint bounds and config:     
        self.showTick = QtGui.QCheckBox("Show configuration")
        self.clickTick = QtGui.QCheckBox("Choose with mouse")
        self.configCombo = QtGui.QComboBox()
        self.configCombo.addItem("initial")
        self.configCombo.addItem("goal")
        self.jointCombo = QtGui.QComboBox()
        self.initq = QtGui.QLineEdit("-2, 0, 0.63, 1, 0, 0, 0")
        self.goalq = QtGui.QLineEdit("3, 0, 0.63, 1, 0, 0, 0")
        vbox.addWidget(self.addWidgetsInHBox([self.bindFunctionToButton("Set joint bounds",\
                self.JointBounds),self.clickTick,]))
        vbox.addWidget(self.addWidgetsInHBox([self.jointCombo, self.configCombo,\
                self.bindFunctionToButton("Add",self.addConfig),\
                self.bindFunctionToButton("Clear",self.clearConfigs)]))
        self.configTree = QtGui.QTreeWidget()

        self.groupbox = QtGui.QGroupBox()
        self.grid2 = QtGui.QGridLayout(self.groupbox)
        self.grid2.setSpacing(10)
        self.jointConfig = []
        self.labels = []
        for i in range(0,4):
            self.jointConfig.append(QtGui.QDoubleSpinBox())
            self.jointConfig[i].setRange(-1000,1000)
            self.jointConfig[i].setSingleStep (0.01)
            self.labels.append(QtGui.QLabel(str(i)+ ":"))

        self.grid2.addWidget (self.labels[0],0,0)
        self.grid2.addWidget (self.jointConfig[0],0,1)
        self.grid2.addWidget (self.labels[1],1,0)
        self.grid2.addWidget (self.jointConfig[1],1,1)
        self.grid2.addWidget (self.labels[2],2,0)
        self.grid2.addWidget (self.jointConfig[2],2,1)
        self.grid2.addWidget (self.labels[3],3,0)
        self.grid2.addWidget (self.jointConfig[3],3,1)
        self.validator = QtGui.QGroupBox()
        self.validator.setStyleSheet("background-color: blue")
        self.grid2.addWidget (self.validator,0,2,4,1)
        vbox.addWidget(self.addWidgetsInHBox([self.configTree, self.groupbox]))

        self.clickTick.clicked.connect(self.setChooseWithMouse)
        self.jointConfig[0].valueChanged.connect(lambda: self.setConfig(0))
        self.jointConfig[1].valueChanged.connect(lambda: self.setConfig(1))
        self.jointConfig[2].valueChanged.connect(lambda: self.setConfig(2))
        self.jointConfig[3].valueChanged.connect(lambda: self.setConfig(3))
        self.jointCombo.currentIndexChanged.connect(self.findSize)

        self.update ()

    def update(self):
        self.ROMaffList.clear()
        self.ROMaffFilterList.clear()
        ROMnames = self.plugin.rbprmClient.rbprm.getROMnames ()
        for name in ROMnames:
            self.ROMaffList.addItem (name)
        filters = self.plugin.rbprmClient.rbprm.getFilter ()
        for fi in filters:
            afffilters = self.plugin.rbprmClient.rbprm.getAffordanceFilter(fi)
            item = QtGui.QTreeWidgetItem()
            item.setText(0,fi)
            if (len(afffilters) > 0):
                for affi in afffilters:
                    child = QtGui.QTreeWidgetItem()
                    child.setText(0,affi)
                    item.addChild(child)
            self.ROMaffFilterList.addTopLevelItem(item)
            self.ROMaffFilterList.expandItem(item)
        self.affTypeList.clear()
        self.affAnalysisOptions.clear()
        affs = self.getAffordanceConfigTypes ()
        self.affAnalysisOptions.addItem ("All types")
        for aff in affs:
            self.affTypeList.addItem(aff)
            self.affAnalysisOptions.addItem (aff)
            if (aff == self.lastAff):
                self.affAnalysisOptions.setCurrentIndex(1 + affs.index(aff))
        self.affAnalysisObjects.clear ()
        objects = self.plugin.basicClient.obstacle.getObstacleNames(False,True)
        self.affAnalysisObjects.addItem ("All objects")
        for obj in objects:
            self.affAnalysisObjects.addItem (obj)
            if (obj == self.lastObject):
                self.affAnalysisObjects.setCurrentIndex(1 + objects.index(obj))
        self.configTree.clear()
        self.jointCombo.clear()
        if hasattr(self.plugin.builder, 'name'):
            joints = self.plugin.basicClient.robot.getJointNames()
            for joint in joints:
                self.jointCombo.addItem(joint)
            if hasattr (self, 'currentJoint'):
                self.jointCombo.setCurrentIndex(self.jointCombo.findText(self.currentJoint))
            if (self.initConfigSet):
                item = QtGui.QTreeWidgetItem()
                item.setText(0,"initial")
                iq = self.plugin.basicClient.problem.getInitialConfig()    
                child = QtGui.QTreeWidgetItem()
                child.setText(0,str(iq))
                item.addChild(child)
                self.configTree.addTopLevelItem(item)
                self.configTree.expandItem(item)
            gqs = self.plugin.basicClient.problem.getGoalConfigs()
            if (len(gqs) > 0):
                item = QtGui.QTreeWidgetItem()
                item.setText(0,"goal")
                for gq in gqs:
                    child = QtGui.QTreeWidgetItem()
                    child.setText(0,str(gq))
                    item.addChild(child)
                self.configTree.addTopLevelItem(item)
                self.configTree.expandItem(item)

    def findSize (self):
        if (hasattr (self.plugin.builder, 'name')):
            joint = str(self.jointCombo.currentText)
            if joint != "":
                self.configSize = self.plugin.basicClient.robot.getJointConfigSize (joint)
                config = self.plugin.basicClient.robot.getJointConfig (joint)
                bounds = self.plugin.basicClient.robot.getJointBounds (joint)
                if (checkBounds (bounds) == False):
                    for i in range (self.configSize):
                        bounds[i*2] = -1000; bounds[i*2 +1] = 1000
                for i in range (len(self.labels)):
                    self.labels[i].setHidden(True)
                    self.jointConfig[i].setHidden(True)
                if self.configSize < 5:
                    for i in range (self.configSize):
                        self.jointConfig[i].setHidden(False)
                        self.jointConfig[i].setRange(bounds[i*2], bounds[i*2 +1])
                        self.jointConfig[i].setValue(config[i])
                        self.labels[i].setHidden(False)
                else:
                    print ("Too many degrees of freedom for joint.")


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
        self.robotdialog = _RobotDialog()
        if self.robotdialog.exec_():
            info = []
            info.append (str(self.robotdialog.Envname.text))
            info.append (str(self.robotdialog.urdfName.text))
            urdfROMs = str(self.robotdialog.urdfromName.text)
            urdfROMs = urdfROMs.replace(" ", "")
            info.append (urdfROMs.split(','))
            info.append (str(self.robotdialog.rootJoint.currentText))
            info.append (str(self.robotdialog.mpkgName.text))
            info.append (str(self.robotdialog.pkgName.text))
            info.append (str(self.robotdialog.urdfSuf.text))
            info.append (str(self.robotdialog.srdfSuf.text))
            self.latestRobotInfo = info
#            self.plugin.builder.loadModel(urdfName, urdfNameRom, rootJointType, \
#                    meshPackageName, packageName, urdfSuffix, srdfSuffix)
            t = Thread(target=self.loadRobot, args=(self, info))
            t.start()
            while t.is_alive():
                QtCore.QCoreApplication.processEvents()
                time.sleep(0.2)
            if (self.plugin.viewCreated == False):
                self.plugin.main.createView("window_hpp_") #TODO: find way of adding view of custom name
                self.plugin.viewCreated = True
            self.addToViewer()
            self.plugin.client.gui.setVisibility(self.plugin.builder.name, "ON")
        self.update()

    def loadRobot (self, tab, info):
        tab.plugin.builder.loadModel (info[1],info[2],info[3],\
                info[4], info[5],info[6], info[7])

    def Environment (self):
        self.envdialog = _EnvDialog()
        if hasattr(self.plugin.builder, 'name'):
            if self.envdialog.exec_():
                info = []
                info.append (str(self.envdialog.pkgName.text))
                info.append (str(self.envdialog.urdfName.text))
                info.append (str(self.envdialog.Envname.text))
                self.latestEnvironmentInfo = info
                t = Thread(target=self.loadEnvironment, args=(self.plugin.r, info))
                t.start()
                while t.is_alive():
                    QtCore.QCoreApplication.processEvents()
                    time.sleep(0.2)
            self.update()
        else:
            print ("Please load ROM robot before environment.")

    def loadEnvironment (self, viewer, info):
        viewer.loadObstacleModel (info[0],info[1],info[2])

    def JointBounds (self):
        if hasattr(self.plugin.builder, 'name'):
            self.boundsdialog = _BoundsDialog(self.plugin)
            if self.boundsdialog.exec_():
                name = str(self.boundsdialog.jointTree.currentItem().text(0))
                bounds = []
                for i in range(2*self.boundsdialog.configSize):
                    bounds.append(self.boundsdialog.bounds[i].value)
                if checkBounds(bounds):
                    self.plugin.builder.setJointBounds (name, bounds)
                    if name == str(self.jointCombo.currentText):
                        for i in range (len(bounds)/2):
                            self.jointConfig[i].setRange(bounds[i*2], bounds[i*2 +1])
                if self.boundsdialog.tick.isChecked():
                    quatbounds = []
                    for i in range (len(self.boundsdialog.quatbounds)):
                        quatbounds.append(self.boundsdialog.quatbounds[i].value)
                    if checkBounds(quatbounds):
                        self.plugin.builder.boundSO3(quatbounds)
                        self.plugin.SO3bounds = quatbounds
        else:
            print ("Please add ROM robot before setting bounds")

    



    def Affordance (self):
        self.affdialog = _AffDialog(self.plugin)
        if self.affdialog.exec_():
            config = [self.affdialog.nm.value, self.affdialog.ntm.value, self.affdialog.ma.value]
            name = str(self.affdialog.affordanceTypes.currentText)
            self.plugin.affClient.affordance.setAffordanceConfig (name, config)

    def clearFilters (self):
        ROMnames = []
        self.plugin.rbprmClient.rbprm.setFilter(ROMnames)
        self.plugin.rbprmClient.rbprm.clearAffordanceFilter()
        self.update()

    def addFilters (self):
        if hasattr(self.plugin.builder, 'name'):
            items = self.ROMaffList.selectedItems()
            affItems = self.affTypeList.selectedItems()
            if (len(items) > 0):
                ROMnames = self.plugin.rbprmClient.rbprm.getFilter()
                for item in items:
                    rom = str(item.text())
                    if rom not in ROMnames:
                        ROMnames.append(rom)
                self.plugin.rbprmClient.rbprm.setFilter(ROMnames)
            affNames = []
            for aff in affItems:
                affNames.append(str(aff.text()))
            for item in items:
                self.plugin.rbprmClient.rbprm.setAffordanceFilter(str(item.text()), affNames)
            self.update()

    def addToViewer (self):
        self.ps = ProblemSolver(self.plugin.builder)
        self.plugin.r = Viewer(self.ps)

    def str2float (self, text):
        str1 = str(text).replace(" ", "")
        str2 = str1.split(',')
        flist = []
        for s in str2:
            flist.append(float(s))
        return flist

    def setConfig(self, idx):
        if hasattr(self.plugin.builder, 'name'):
            joint = str(self.jointCombo.currentText)
            config = self.plugin.basicClient.robot.getJointConfig(joint)
            configSize = self.plugin.basicClient.robot.getJointConfigSize(joint)
            if configSize < 5:
                config[idx] = self.jointConfig[idx].value
            self.plugin.basicClient.robot.setJointConfig(joint, config)
            conf = self.plugin.basicClient.robot.getCurrentConfig()
            self.plugin.r(conf)
            # if affordance analysis done and robot is the ROM robot, not fullbody, validate config:
            if (self.analysed and self.plugin.basicClient.robot.getRobotName() == self.plugin.builder.name):
                if self.plugin.rbprmClient.rbprm.validateConfiguration(conf):
                    self.validator.setStyleSheet("background-color: green")
                else:
                    self.validator.setStyleSheet("background-color: red")

    def addConfig (self):
        conf = self.plugin.builder.getCurrentConfig()
        if str(self.configCombo.currentText) == 'initial':
            self.plugin.basicClient.problem.setInitialConfig(conf)
            self.plugin.builder.setCurrentConfig(conf)
            self.q_init = conf
            self.initConfigSet = True
        else:
            self.plugin.basicClient.problem.addGoalConfig(conf)
            self.q_goal = conf
        self.currentJoint = self.jointCombo.currentText
        self.update ()

    def clearConfigs (self):
        if str(self.configCombo.currentText) == 'initial':
            configSize = self.plugin.basicClient.robot.getConfigSize()
            conf = [0]*configSize
            self.plugin.basicClient.problem.setInitialConfig(conf)
            self.initConfigSet = False
        else:
            self.plugin.basicClient.problem.resetGoalConfigs()
        self.currentJoint = self.jointCombo.currentText
        self.update()

    def qvector2float (self, p):
        conf =  [p.x(), p.y(), p.z()]
        return conf

    def spinConfig (self, conf):
        if len (conf) > 2:
            # need to check rootjoint type -> this is only valid for freeflyer (and xy planar?)
            joints = self.plugin.basicClient.robot.getJointNames()
            # first element is base as joint names in the same order as in the configuration
            pos = self.plugin.basicClient.robot.getJointConfig(joints[0])
            pos[0:2] = conf[0:2] # do not change z position
            self.plugin.basicClient.robot.setJointConfig(joints[0], pos)
            idx = self.jointCombo.findText(joints[0])
            if idx >= 0:
                self.jointCombo.setCurrentIndex(idx)
            q = self.plugin.basicClient.robot.getCurrentConfig()
            self.currentJoint = self.jointCombo.currentText
            self.update ()
            self.plugin.r(q)

    def setChooseWithMouse (self):
        self.plugin.chooseWithMouse = True

    def colour_picker (self):
        colour = QtGui.QColorDialog.getColor()
        self.colour= [colour.redF(), colour.greenF(), colour.blueF(), colour.alphaF()]
        colourInt = [colour.red(), colour.green(), colour.blue(), colour.alpha()]
        colour = str(colourInt); colour = colour.replace("[",""); colour = colour.replace("]","")
        self.colourBox.setStyleSheet("background-color: rgba("+colour+")")

    def createGroup (self):
        self.plugin.client.gui.createGroup(str(self.nodeName.text))
        self.groupNodes.addItem(self.nodeName.text)
        self.refreshBodyTree()

    def addToGroup (self):
        self.plugin.client.gui.addToGroup(str(self.nodeName.text), str(self.groupNodes.currentText))
        self.refreshBodyTree()

    def affordanceAnalysis (self):
        objectname = str(self.affAnalysisObjects.currentText)
        self.lastObject = objectname
        affType = str(self.affAnalysisOptions.currentText)
        self.lastAff = affType
        if objectname == "All objects":
            objectname = ""
         #   self.plugin.affClient.affordance.analyseAll ()
            t = Thread(target=self.analyseAll)
        else:
            t = Thread(target=self.analyseObject, args=(objectname,))
         #   self.plugin.affClient.affordance.analyseObject (objectname)
        t.start()
        while t.is_alive():
            QtCore.QCoreApplication.processEvents()
            time.sleep(0.2)
        if affType == "All types":
            affordances = self.getAffordanceTypes ()
            for aff in affordances:
                 self.visualiseAffordances(aff, self.colour, objectname)
        else:
            self.visualiseAffordances(affType, self.colour, objectname)
        self.analysed = True
        self.update ()

    def analyseAll (self):
        self.plugin.affClient.affordance.analyseAll ()

    def analyseObject (self, objectname):
        self.plugin.affClient.affordance.analyseObject (objectname)

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
          for groupNode in groupNodes :
              self.plugin.client.gui.removeFromGroup(groupNode,'hpp-gui')
              self.plugin.client.gui.addToGroup(groupNode,'hpp-gui')

    def deleteNode (self, nodeName, all):
        return self.plugin.client.gui.deleteNode (nodeName, all)

    def getAffordanceTypes (self):
        return self.plugin.affClient.affordance.getAffordanceTypes ()

    def getAffordanceConfigTypes (self):
        return self.plugin.affClient.affordance.getAffordanceConfigTypes ()

    def getAffRefObstacles (self, affType):
        return self.plugin.affClient.affordance.getAffRefObstacles (affType)

    def deleteAffordancesByType (self):
        objectname = str(self.affAnalysisObjects.currentText)
        self.lastObject = objectname
        affType = str(self.affAnalysisOptions.currentText)
        self.lastAff = affType
        if objectname == "All objects":
            objectname = ""
        if affType == "All types":
            affType == ""
            affs = self.getAffordanceTypes()
            for aff in affs:
                self.deleteAffordancesByTypeFromViewer (aff, objectname)
            self.plugin.affClient.affordance.deleteAffordances(objectname)
        else:
            self.deleteAffordancesByTypeFromViewer (affType, objectname)
            self.plugin.affClient.affordance.deleteAffordancesByType (affType, objectname)
        self.update ()

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

class _BoundsDialog(QtGui.QDialog):
    def __init__(self, plugin, parent=None):
        super(_BoundsDialog, self).__init__(parent)
        self.plugin = plugin
        self.grid = QtGui.QGridLayout()
        self.grid.setSpacing(10)
        self.setGeometry(440,418, 450, 450)
        self.setWindowTitle('Input dialog')
        # Create widgets
        label = QtGui.QLabel("Translation bounds for robot joints")
        self.jointTree = QtGui.QTreeWidget()
        self.boundsTree = QtGui.QTreeWidget()
        self.groupbox0 = QtGui.QGroupBox()
        self.grid1 = QtGui.QGridLayout(self.groupbox0)
        self.bounds = []
        for i in range(0,8):
            self.bounds.append(QtGui.QDoubleSpinBox())
            self.bounds[i].setRange(-1000,1000)
            self.bounds[i].setSingleStep (0.05)
        self.labels = []
        for i in range (0,4):
            self.labels.append(QtGui.QLabel("variable " + str(i)))
        self.tick = QtGui.QCheckBox("Set rotation bounds for robot base in rbprm shooter")
        self.tick.setCheckable(True)

        # add widgets to grid
        self.grid.addWidget (label, 0,0)
        self.grid.addWidget (self.jointTree, 1,0,3,1)
        self.grid.addWidget (self.boundsTree, 1,1,3,1)
        self.grid1.addWidget (self.labels[0], 0,0)
        self.grid1.addWidget (self.bounds[0], 0,1)
        self.grid1.addWidget (self.bounds[1], 0,2)
        self.grid1.addWidget (self.labels[1], 1,0)
        self.grid1.addWidget (self.bounds[2], 1,1)
        self.grid1.addWidget (self.bounds[3], 1,2)
        self.grid1.addWidget (self.labels[2], 2,0)
        self.grid1.addWidget (self.bounds[4], 2,1)
        self.grid1.addWidget (self.bounds[5], 2,2)
        self.grid1.addWidget (self.labels[3], 3,0)
        self.grid1.addWidget (self.bounds[6], 3,1)
        self.grid1.addWidget (self.bounds[7], 3,2)
        self.grid.addWidget (self.tick, 8,0,1,2)

        # create grid 2 and its widgets
        self.groupbox = QtGui.QGroupBox()
        self.grid2 = QtGui.QGridLayout(self.groupbox)
        self.grid2.setSpacing(10)
        self.quatbounds = []
        for i in range(0,6):
            self.quatbounds.append(QtGui.QDoubleSpinBox())
            self.quatbounds[i].setRange(-1000,1000)
            self.quatbounds[i].setSingleStep (0.05)
        self.quatlabels = []
        for i in range (0,3):
            self.quatlabels.append(QtGui.QLabel("variable " + str(i)))
        if (checkBounds(self.plugin.SO3bounds)):
            for i in range (len(self.quatbounds)):
                self.quatbounds[i].setValue(self.plugin.SO3bounds[i])
        else:
            self.quatbounds[0].setValue(-0.4); self.quatbounds[1].setValue(0.4); self.quatbounds[2].setValue(-3)
            self.quatbounds[3].setValue(3); self.quatbounds[4].setValue(-3); self.quatbounds[5].setValue(3)

        self.OKbutton = QtGui.QPushButton("Set bounds")
        self.CANCELbutton = QtGui.QPushButton("Cancel")

        # add widgets to grid2
        #self.grid2.addWidget (label, 0,0)
        self.grid2.addWidget (self.quatlabels[0], 1,0)
        self.grid2.addWidget (self.quatbounds[0], 1,1)
        self.grid2.addWidget (self.quatbounds[1], 1,2)
        self.grid2.addWidget (self.quatlabels[1], 2,0)
        self.grid2.addWidget (self.quatbounds[2], 2,1)
        self.grid2.addWidget (self.quatbounds[3], 2,2)
        self.grid2.addWidget (self.quatlabels[2], 3,0)
        self.grid2.addWidget (self.quatbounds[4], 3,1)
        self.grid2.addWidget (self.quatbounds[5], 3,2)
                
        self.grid.addWidget (self.groupbox0,4,0,4,2)
        self.grid.addWidget (self.groupbox,9,0,3,2)
        self.groupbox.setHidden(True)
        self.grid.addWidget (self.CANCELbutton, 12,0,1,1)
        self.grid.addWidget (self.OKbutton, 12,1,1,1)

        # Set dialog layout
        self.setLayout(self.grid)
        # Add button signal to showDetails slot
        self.jointTree.currentItemChanged.connect(self.findSize)
        self.tick.clicked.connect(lambda: self.hideWidget(self.groupbox, self.tick))
        self.OKbutton.clicked.connect(self.load)
        self.CANCELbutton.clicked.connect(self.cancel)
        self.update()

    def child (self, item):
        children = self.plugin.basicClient.robot.getChildJointNames(str(item.text(0)))
        if len(children) > 0:
            for ch in children:
                childItem = QtGui.QTreeWidgetItem()
                childItem.setText(0,str(ch))
                item.addChild(childItem)
                self.child (childItem)
        return item

    def expand (self, tree, topItem):
        child_count = topItem.childCount()
        for ch in range (child_count):
            item = topItem.child(ch)
            self.expand(tree, item)
        tree.expandItem(topItem)

    # methods
    def update(self):
        self.jointTree.clear()
        self.boundsTree.clear()
        joints = self.plugin.basicClient.robot.getAllJointNames()
        if len(joints) > 0:
            joint = joints[0]
            it = QtGui.QTreeWidgetItem()
            it.setText(0,joint)
            item = self.child(it)
            self.jointTree.addTopLevelItem(item)
            self.expand(self.jointTree, item)
            if hasattr (self, 'currentJoint'):
                self.jointTree.setCurrentItem(self.currentJoint)
            else:
                self.jointTree.setCurrentItem(item)
        for joint in joints:
            bounds = self.plugin.basicClient.robot.getJointBounds (joint)
            if (len(bounds) > 1):
                if (checkBounds(bounds)):
                    item = QtGui.QTreeWidgetItem()
                    item.setText(0,joint)
                    child = QtGui.QTreeWidgetItem()
                    child.setText(0,str(bounds))
                    item.addChild(child)
                    self.boundsTree.addTopLevelItem(item)
                    self.boundsTree.expandItem(item)
        if (checkBounds(self.plugin.SO3bounds)):
            item = QtGui.QTreeWidgetItem()
            item.setText(0,'So3bounds')
            child = QtGui.QTreeWidgetItem()
            child.setText(0,str(self.plugin.SO3bounds))
            item.addChild(child)
            self.boundsTree.addTopLevelItem(item)
            self.boundsTree.expandItem(item)

    def load (self):
            self.accept()
    def cancel (self):
            self.reject()
    def hideWidget (self,widget, button):
        if button.isChecked():
            widget.setHidden(False)
        else:
            widget.setHidden(True)

    def findSize (self):
        self.currentJoint = self.jointTree.currentItem()
        joint = str(self.jointTree.currentItem().text(0))
        self.configSize = self.plugin.basicClient.robot.getJointConfigSize (joint)
        bounds = self.plugin.basicClient.robot.getJointBounds (joint)
        if self.configSize < 5:
            for i in range (0,len(self.labels)):
                self.labels[i].setHidden(True)
            for i in range (0,len(self.bounds)):
                self.bounds[i].setHidden(True)
            for i in range (0,self.configSize):
                self.bounds[i*2].setHidden(False)
                self.bounds[i*2].setValue(bounds[i*2])
                self.bounds[i*2+1].setHidden(False)
                self.bounds[i*2+1].setValue(bounds[i*2+1])
                self.labels[i].setHidden(False)
        else:
            print ("Too many degrees of freedom for joint.")
        # TODO: setting certain values is only for now (quick setup)
        if (checkBounds (bounds) == False):
            self.bounds[0].setValue(-2); self.bounds[1].setValue(5); self.bounds[2].setValue(-1)
            self.bounds[3].setValue(1); self.bounds[4].setValue(0.3); self.bounds[5].setValue(4)

    def hideWidgetCustom (self, listItem):
        joint = str(listItem.text())
        if (joint.find ('base_joint') != -1):
            self.tick.setHidden(False)
        else:
            self.tick.setHidden(True)

class _LimbTab (QtGui.QWidget):
    def __init__(self, parent, plugin):
        super(_LimbTab, self).__init__ (parent)
        self.grid = QtGui.QGridLayout()
        self.grid.setSpacing(10)
        # Create widgets
        label = QtGui.QLabel("Choose predefined model or provide custom description.")
        self.model = QtGui.QComboBox()
        self.model.editable = False
        text1 = QtGui.QLabel('Choose predefined model')
        text2 = QtGui.QLabel('Limb name')
        self.legId = QtGui.QLineEdit("rfleg")
        self.button = QtGui.QPushButton("Details")
        self.button.setCheckable(True)

        # add widgets to grid
        self.grid.addWidget (label, 0,0)
        self.grid.addWidget (text1, 1,0)
        self.grid.addWidget (self.model, 1,1)
        self.grid.addWidget (text2,2,0,2,1)
        self.grid.addWidget (self.legId, 2,1,2,1)
        self.grid.addWidget (self.button, 4,1,1,1)
        # create grid 2 and its widgets
        self.groupbox = QtGui.QGroupBox()
        self.grid2 = QtGui.QGridLayout(self.groupbox)
        self.grid2.setSpacing(10)
        text3 = QtGui.QLabel('Constraint type')
        self.cType = QtGui.QComboBox()
        self.cType.addItem("_3_DOF")
        text4 = QtGui.QLabel('First joint in URDF file')
        self.leg = QtGui.QLineEdit("rf_haa_joint")
        text5 = QtGui.QLabel('Last joint in URDF file')
        self.foot = QtGui.QLineEdit("rf_foot_joint")
        text6 = QtGui.QLabel('Offset between last joint and contact surface')
        self.offset = QtGui.QLineEdit("0,-0.021,0")
        text7 = QtGui.QLabel('Contact surface direciton for limb in rest pose')
        self.normal = QtGui.QLineEdit("0,1,0")
        text8 = QtGui.QLabel('Rectangular-contact-surface dimensions')
        self.legxlegy = QtGui.QLineEdit("0.02,0.02")
        text9 = QtGui.QLabel('Heuristic')
        self.heur = QtGui.QComboBox()
        self.heur.addItem ("manipulability")
        text10 = QtGui.QLabel ("Octree resolution")
        self.res = QtGui.QDoubleSpinBox()
        self.res.setSingleStep (0.05)
        self.res.setValue (0.1)
        text11 = QtGui.QLabel ("Number of samples")
        self.nbSamples = QtGui.QSpinBox()
        self.nbSamples.setMaximum(30000)
        self.nbSamples.setValue(20000)


        # add widgets to grid2
        self.grid2.addWidget (text3, 0,0)
        self.grid2.addWidget (self.cType, 0,1)
        self.grid2.addWidget (text4, 1,0)
        self.grid2.addWidget (self.leg, 1,1)
        self.grid2.addWidget (text5, 2,0)
        self.grid2.addWidget (self.foot, 2,1)
        self.grid2.addWidget (text6, 3,0)
        self.grid2.addWidget (self.offset, 3,1)
        self.grid2.addWidget (text7, 4,0)
        self.grid2.addWidget (self.normal, 4,1)
        self.grid2.addWidget (text8, 5,0)
        self.grid2.addWidget (self.legxlegy, 5,1)
        self.grid2.addWidget (text9, 6,0)
        self.grid2.addWidget (self.heur, 6,1)
        self.grid2.addWidget (text10, 7,0)
        self.grid2.addWidget (self.res, 7,1)
        self.grid2.addWidget (text11, 8,0)
        self.grid2.addWidget (self.nbSamples, 8,1)
       

        self.grid.addWidget (self.groupbox,5,0,3,2)
        self.groupbox.setHidden(True)

        # Set dialog layout
        self.setLayout(self.grid)
        # Add button signal to showDetails slot
        self.button.clicked.connect(lambda: self.hideWidget(self.groupbox, self.button))

    def hideWidget (self,widget, button):
        if button.isChecked():
            widget.setHidden(False)
        else:
            widget.setHidden(True)

class _LimbDialog(QtGui.QDialog):
    def __init__(self, parent=None):
        super(_LimbDialog, self).__init__(parent)
        self.tabWidget = QtGui.QTabWidget()
        self.setGeometry(440,470, 450, 490)
        self.setWindowTitle('Input dialog')
       #  self.tabWidget.addTab(self.tab, "tab")
        self.tabs = [0]*4
        text1 = QtGui.QLabel("Number of limbs")
        self.limbs = QtGui.QSpinBox()
        self.limbs.setValue(4)
        self.limbs.setMaximum(10)
        self.OKbutton = QtGui.QPushButton("Load all limbs")
        self.CANCELbutton = QtGui.QPushButton("Cancel")
        # Set dialog layout
        layout = QtGui.QVBoxLayout()
        layout.addWidget (self.addWidgetsInHBox([text1, self.limbs]))
        layout.addWidget(self.tabWidget)
        layout.addWidget (self.addWidgetsInHBox([self.CANCELbutton, self.OKbutton]))

        self.setLayout(layout)
        self.OKbutton.clicked.connect(self.load)
        self.CANCELbutton.clicked.connect(self.cancel)
        self.limbs.valueChanged.connect(self.update)
        self.update()
    # methods

    def update (self):
        self.tabWidget.clear()
        self.tabs = [0]*self.limbs.value
        for i in range(0,self.limbs.value):
            self.tabs[i] = _LimbTab (self,self)
            self.tabWidget.addTab(self.tabs[i], "limb" + str(i+1) )
            if i == 1:
                self.tabs[i].legId.setText("lhleg")
                self.tabs[i].leg.setText("lh_haa_joint")
                self.tabs[i].foot.setText("lh_foot_joint")
                self.tabs[i].offset.setText("0,0.021,0")
                self.tabs[i].normal.setText("0,-1,0")
            if i == 2:
                self.tabs[i].legId.setText("rhleg")
                self.tabs[i].leg.setText("rh_haa_joint")
                self.tabs[i].foot.setText("rh_foot_joint")
            if i == 3:
                self.tabs[i].legId.setText("lfleg")
                self.tabs[i].leg.setText("lf_haa_joint")
                self.tabs[i].foot.setText("lf_foot_joint")
                self.tabs[i].offset.setText("0,0.021,0")
                self.tabs[i].normal.setText("0,-1,0")

    
    def load (self):
            self.accept()
    def cancel (self):
            self.reject()
    def hideWidget (self,widget, button):
        if button.isChecked():
            widget.setHidden(False)
        else:
            widget.setHidden(True)

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


class _EnvDialog(QtGui.QDialog):
    def __init__(self, parent=None):
        super(_EnvDialog, self).__init__(parent)
        self.settings = Settings()
        self.infos = self.read()
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
        self.grid.addWidget (label, 0,0,1,2)
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

    def read(self):
        return self.settings.readEnvironments()
        
class _RobotDialog(_EnvDialog):
    def __init__(self, parent=None):
        super(_RobotDialog, self).__init__(parent)
        #self.grid2.removeWidget(self.envText6)
        #self.grid2.removeWidget(self.urdfromName)
        self.infos = self.settings.readRobots()
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

    def read(self):
        return self.settings.readRobots()

class _FullBodyDialog (_RobotDialog):
     def __init__(self, parent=None):
        super(_FullBodyDialog, self).__init__(parent)

        self.Envname.setText("hyq")
        self.urdfName.setText("hyq")
        self.pkgName.setText("hyq_description")
        self.mpkgName.setText("hyq_description")
        self.envText6.setHidden(True)
        self.urdfromName.setHidden(True)

        def read(self):
            return self.settings.readFullbodies()

class _AffDialog(QtGui.QDialog):
    def __init__(self, plugin, parent=None):
        super(_AffDialog, self).__init__(parent)
        self.plugin = plugin
        self.grid = QtGui.QGridLayout()
        self.grid.setSpacing(10)
        self.setGeometry(440,418, 450, 450)
        self.setWindowTitle('Affordance settings')
        # Create widgets
        self.affordanceTypes = QtGui.QComboBox(self)
        self.affordanceTypes.clear()
        affordances = self.getAffordanceConfigTypes()
        for aff in affordances:
            self.affordanceTypes.addItem (aff)

        self.ntm = QtGui.QDoubleSpinBox()
        self.ntm.setRange(0,1)
        self.nm = QtGui.QDoubleSpinBox()
        self.nm.setRange(0,1)
        self.ma = QtGui.QDoubleSpinBox()
        self.ma.setRange(0,10)

        self.ntm.setSingleStep (0.01)
        self.nm.setSingleStep (0.01)
        self.ma.setSingleStep (0.01)
        self.ntm.setValue (0.03)
        self.nm.setValue (0.03)
        self.ma.setValue (0.05)
        text1 = QtGui.QLabel('Affordance type')
        text2 = QtGui.QLabel('Neighbouring-triangle margin')
        text3 = QtGui.QLabel('Normal margin')
        text4 = QtGui.QLabel('Minimum area')
        self.OKbutton = QtGui.QPushButton("Change Settings")
        self.CANCELbutton = QtGui.QPushButton("Cancel")
        # add widgets to grid
        self.grid.addWidget (text1, 0,0)
        self.grid.addWidget (self.affordanceTypes, 0,1)
        self.grid.addWidget (text2, 1,0)
        self.grid.addWidget (self.ntm, 1,1)
        self.grid.addWidget (text3, 2,0)
        self.grid.addWidget (self.nm, 2,1)
        self.grid.addWidget (text4, 3,0)
        self.grid.addWidget (self.ma, 3,1)
        self.grid.addWidget (self.CANCELbutton, 4,0)
        self.grid.addWidget (self.OKbutton, 4,1)

        # Set dialog layout
        self.setLayout(self.grid)
        self.affordanceTypes.currentIndexChanged.connect(self.update)
        self.OKbutton.clicked.connect(self.load)
        self.CANCELbutton.clicked.connect(self.cancel)
        self.update()

    def update(self):
        aff = str(self.affordanceTypes.currentText)
        config = self.plugin.affClient.affordance.getAffordanceConfig(aff)
        self.nm.setValue(config[0])
        self.ntm.setValue(config[1])
        self.ma.setValue(config[2])
            
    def getAffordanceConfigTypes (self):
        return self.plugin.affClient.affordance.getAffordanceConfigTypes ()

    def load (self):
            self.accept()
    def cancel (self):
            self.reject()

#------------------------------------------------------------------end _Dialogs

#------------------------------------------------------------------start _AffCreator
class _RbprmInterp (QtGui.QWidget):
    def __init__(self, parent, plugin):
        super(_RbprmInterp, self).__init__ (parent)
        self.plugin = plugin
        self.plugin.SO3bounds = [1,0,1,0,1,0]
        self.configs = []
        self.tolerance = -1
        self.startPoint = 0
        self.paused = False
        self.stopped = False
        self.interrupted = False
       # self.exampleThread = QtCore.QThread()
        vbox1 = QtGui.QVBoxLayout(self)
        gridW = QtGui.QWidget()#(self)
        grid = QtGui.QGridLayout(gridW)
        
        self.groupbox = QtGui.QGroupBox()
        self.grid2 = QtGui.QGridLayout(self.groupbox)
        optimiserLabel = QtGui.QLabel("Choose Path Optimisers:")
        self.optimiserList = QtGui.QListWidget()
        self.optimiserList.setSelectionMode(QtGui.QAbstractItemView.ExtendedSelection)
        self.chosenTree = QtGui.QTreeWidget()
        shooterLabel = QtGui.QLabel("Configuration Shooter:")
        self.shooters = QtGui.QComboBox()
        validationLabel = QtGui.QLabel("Path Validation method:")
        self.validations = QtGui.QComboBox()
        toleranceLabel = QtGui.QLabel("Validation tolerance:")
        self.validationTolerance = QtGui.QDoubleSpinBox()
        self.validationTolerance.setSingleStep (0.01)
        self.validationTolerance.setValue(0.05)
        self.validate = self.bindFunctionToButton("Validate settings", self.validateSettings)
        self.clearOptimisers = self.bindFunctionToButton("Clear path optimisers", self.clearOptimisers)
        self.grid2.addWidget(optimiserLabel, 0,0)
        self.grid2.addWidget(self.optimiserList, 1,0)
        self.grid2.addWidget(self.chosenTree, 1,1)
        self.grid2.addWidget(shooterLabel, 2,0)
        self.grid2.addWidget(self.shooters, 2,1)
        self.grid2.addWidget(validationLabel, 3,0)
        self.grid2.addWidget(self.validations, 3,1)
        self.grid2.addWidget(toleranceLabel, 4,0)
        self.grid2.addWidget(self.validationTolerance, 4,1)
        self.grid2.addWidget(self.validate, 5,1)
        self.grid2.addWidget(self.clearOptimisers, 5,0)
        vbox1.addWidget(self.groupbox)
        self.colourBox = QtGui.QGroupBox()
        self.solveStatus = QtGui.QLabel()
        self.solveStatus.setAlignment(QtCore.Qt.AlignCenter)
        self.solveStatus.setStyleSheet("background-color: blue")
        vbox1.addWidget(self.addWidgetsInHBox([self.bindFunctionToButton("Solve", self.solve),\
                self.solveStatus, self.bindFunctionToButton("Interrupt", self.interruptPathPlanning)]))

        self.space = QtGui.QLabel("")
        self.computeStatus = QtGui.QLabel()
        self.computeStatus.setAlignment(QtCore.Qt.AlignCenter)
        self.computeStatus.setStyleSheet("background-color: blue")
        vbox1.addWidget(self.addWidgetsInHBox([self.bindFunctionToButton("Load full-body model",\
                self.Fullbody), self.bindFunctionToButton("Load limb models", self.Limb)]))
        vbox1.addWidget(self.addWidgetsInHBox([self.bindFunctionToButton("Compute contacts",\
                self.computeContacts), self.computeStatus, self.space]))
        self.slider = QtGui.QSlider(QtCore.Qt.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(100)
        self.slider.setValue(0)
        self.slider.setTickInterval(1)
        
        self.play = QtGui.QPushButton()
        self.play.setIcon(self.style().standardIcon(QtGui.QStyle.SP_MediaPlay))
        self.stop = QtGui.QPushButton()
        self.stop.setIcon(self.style().standardIcon(QtGui.QStyle.SP_MediaStop))
        self.pause = QtGui.QPushButton()
        self.pause.setIcon(self.style().standardIcon(QtGui.QStyle.SP_MediaPause))
        vbox1.addWidget(self.addWidgetsInHBox([self.play, self.stop, self.pause, self.slider]))
        self.play.setDisabled(True)

        self.play.clicked.connect(self.playConfigs)
        self.stop.clicked.connect(self.stopPlayer)
        self.pause.clicked.connect(self.pausePlayer)
        self.slider.valueChanged.connect(self.draw)
        
        vbox1 = QtGui.QVBoxLayout(self)
        self.update ()

    def update(self):
        opts = self.plugin.basicClient.problem.getAvailable("PathOptimizer")
        vals = self.plugin.basicClient.problem.getAvailable("PathValidation")
        shs = self.plugin.basicClient.problem.getAvailable("ConfigurationShooter")
        self.optimiserList.clear()
        self.validations.clear()
        self.shooters.clear()
        item = []
        for opt in opts:
            item = QtGui.QListWidgetItem (opt)
            self.optimiserList.addItem(item)
        self.optimiserList.setCurrentItem(item)
        for val in vals:
            self.validations.addItem(val)
        self.validations.setCurrentIndex(vals.index(val))
        for sh in shs:
            self.shooters.addItem(sh)
        self.shooters.setCurrentIndex(shs.index(sh))
        opts = self.plugin.basicClient.problem.getSelected("PathOptimizer")
        vals = self.plugin.basicClient.problem.getSelected("PathValidation")
        shs = self.plugin.basicClient.problem.getSelected("ConfigurationShooter")
        self.chosenTree.clear()
        item = QtGui.QTreeWidgetItem()
        item.setText(0,"Path optimisers")
        for opt in opts:
            child = QtGui.QTreeWidgetItem()
            child.setText(0,opt)
            item.addChild(child)
        self.chosenTree.addTopLevelItem(item)
        self.chosenTree.expandItem(item)
        item = QtGui.QTreeWidgetItem()
        item.setText(0,"Configuration shooter")
        for sh in shs:
            child = QtGui.QTreeWidgetItem()
            child.setText(0,sh)
            item.addChild(child)
        self.chosenTree.addTopLevelItem(item)
        self.chosenTree.expandItem(item)
        item = QtGui.QTreeWidgetItem()
        item.setText(0,"Path validation")
        for val in vals:
            child = QtGui.QTreeWidgetItem()
            child.setText(0,val)
            item.addChild(child)
            if self.tolerance > 0:
                child = QtGui.QTreeWidgetItem()
                child.setText(0,str(self.tolerance))
                item.addChild(child)
        self.chosenTree.addTopLevelItem(item)
        self.chosenTree.expandItem(item)
        self.slider.setMaximum(len(self.configs)-1)
        
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

    def Fullbody (self):
        self.resetStatus()
        self.fbdialog = _FullBodyDialog()
        if self.fbdialog.exec_():
            info = []
            info.append (str(self.fbdialog.Envname.text))
            info.append (str(self.fbdialog.urdfName.text))
            info.append (str(self.fbdialog.rootJoint.currentText))
            info.append (str(self.fbdialog.mpkgName.text))
            info.append (str(self.fbdialog.pkgName.text))
            info.append (str(self.fbdialog.urdfSuf.text))
            info.append (str(self.fbdialog.srdfSuf.text))
            self.latestFullbodyInfo = info
            t = Thread(target=self.loadFullbody, args=(self, info))
            t.start()
            while t.is_alive():
                QtCore.QCoreApplication.processEvents()
                time.sleep(0.2)
            self.addToViewer()
            self.plugin.client.gui.setVisibility(self.plugin.builder.name, "OFF")
            self.update()

    def loadFullbody (self, tab, info):
        tab.plugin.fullbody.loadFullBodyModel (info[1],info[2],info[3],\
                info[4], info[5],info[6])

    def Limb (self):
        self.limbdialog = _LimbDialog()
        if self.limbdialog.exec_():
            tabs = self.limbdialog.tabs
            infos = []
            self.legIds = []
            for tab in tabs:
                info = []
                info.append (str(tab.legId.text))
                self.legIds.append(info[0])     
                info.append (str(tab.leg.text))
                info.append (str(tab.foot.text))
                info.append (self.str2float(str(tab.offset.text)))
                info.append (self.str2float(str(tab.normal.text)))
                info.append (self.str2float(str(tab.legxlegy.text)))
                info.append (int(tab.nbSamples.value))
                info.append (str(tab.heur.currentText))
                info.append (float(tab.res.value))
                info.append (str(tab.cType.currentText))
                infos.append(info)
                self.latesLimbInfos = infos
            t = Thread(target=self.loadLimbs, args=(self, infos))
            t.start()
            while t.is_alive():
                QtCore.QCoreApplication.processEvents()
                time.sleep(0.2)
            self.plugin.r(self.plugin.fullbody.getCurrentConfig())
            self.update()

    def loadLimbs(self, tab, infos):
        for info in infos:
            tab.plugin.fullbody.addLimb(info[0],info[1],info[2],info[3],info[4], info[5][0],\
                    info[5][1], info[6], info[7], info[8], info[9])

    def computeContacts (self):
        self.resetStatus()
        if self.plugin.rbprmPath.initConfigSet == False:
            print ("Please set initial configuration and solve before computing contacts.")
        else:
            goals = self.plugin.basicClient.problem.getGoalConfigs()
            if (len (goals)) < 1:
                print ("Please add goal configurationand solve before computing contacts.")
            else:
                #TODO check that init & goal not the same as this leads to segmentation fault
                self.computeStatus.setText("Computing")
                self.computeStatus.setStyleSheet("background-color: yellow")
                t = Thread(target=self.contacts, args=(self,\
                        self.plugin.rbprmPath.q_init, self.plugin.rbprmPath.q_goal))
                t.start()
                while t.is_alive():
                    QtCore.QCoreApplication.processEvents()
                    time.sleep(0.2)
                self.plugin.r(self.q_init)
                self.computeStatus.setText("Done")
                self.computeStatus.setStyleSheet("background-color: green")
                self.play.setDisabled(False)
                self.update()

    def contacts (self, tab, q_i, q_g):
        q_init = tab.plugin.fullbody.getCurrentConfig()
        q_init[0:7] = q_i[0:7]
        q_goal = tab.plugin.fullbody.getCurrentConfig();
        q_goal[0:7] = q_g[0:7]
        tab.plugin.fullbody.setCurrentConfig (q_init)
        tab.q_init = tab.plugin.fullbody.generateContacts(q_init, [0,0,1])
        tab.plugin.fullbody.setCurrentConfig (q_goal)
        tab.q_goal = tab.plugin.fullbody.generateContacts(q_goal, [0,0,1])
        tab.plugin.fullbody.setStartState(q_init,[])
        tab.plugin.fullbody.setEndState(q_goal,tab.legIds)        
        configs = self.plugin.fullbody.interpolate(0.1, 1, 0)
        tab.configs = configs[1:len(configs)-1]

    def playConfigs (self):
        self.resetStatus()
        self.stopped = False
        self.paused = False
        start = self.startPoint
        if len (self.configs) > 0:
            for i in range(start,len(self.configs) -1):
                QtCore.QCoreApplication.processEvents()
                self.slider.setValue(i)
                if (self.paused):
                    self.startPoint = i
                    return
                elif (self.stopped):
                    self.startPoint = 0
                    self.slider.setValue(1)
                    return
                time.sleep (0.2)
        self.startPoint = 0

    def stopPlayer (self):
        self.stopped = True
        self.startPoint = 0
        self.slider.setValue(1)

    def pausePlayer (self):
        self.paused = True

    def addToViewer (self):
        self.ps = ProblemSolver(self.plugin.fullbody)
        self.plugin.r = Viewer(self.ps)

    def str2float (self,text):
        str1 = str(text).replace(" ", "")
        str2 = str1.split(',')
        flist = []
        for s in str2:
            flist.append(float(s))
        return flist
    
    def draw(self, i):
        if len(self.configs) > 0:
            config = self.configs[i]
            self.plugin.fullbody.draw(config, self.plugin.r)

    def showConfig(self):
        if str(self.configCombo.currentText) == 'initial':
            q = self.str2float(self.initq.text)
        else:
            q = self.str2float(self.goalq.text)
        if len(q) == self.plugin.basicClient.robot.getConfigSize():
            self.plugin.r (q)
        self.update()

    def resetStatus (self):
        self.interrupted = False
        self.solveStatus.setText("")
        self.solveStatus.setStyleSheet("background-color: blue")
        self.computeStatus.setText("")
        self.computeStatus.setStyleSheet("background-color: blue")

    def validateSettings (self):
        self.plugin.basicClient.problem.addPathOptimizer (str(self.optimiserList.currentItem().text()))
        self.plugin.basicClient.problem.selectConFigurationShooter (str(self.shooters.currentText))
        self.plugin.basicClient.problem.selectPathValidation (str(self.validations.currentText),\
                float(self.validationTolerance.value))
        self.tolerance = float(self.validationTolerance.value)
        self.update()

    def clearOptimisers (self):
        self.plugin.basicClient.problem.clearPathOptimizers()
        self.update()

    def solve (self):
        self.resetStatus()
        if self.plugin.rbprmPath.initConfigSet == False:
            print ("Please set initial configuration before solving.")
        else:
            goals = self.plugin.basicClient.problem.getGoalConfigs()
            if (len (goals)) < 1:
                print ("Please add goal configuration before solving.")
            else:
                init = self.plugin.basicClient.problem.getInitialConfig()
                for i in range (len(goals)):
                    if equal(init, goals[i]):
                        print ("Initial and goal configuration should not be equal.")
                        return
                self.solveStatus.setText("Solving")
                self.solveStatus.setStyleSheet("background-color: yellow")
                t = Thread(target=self.solver, args=(self,))
                t.start()
                while t.is_alive():
                    QtCore.QCoreApplication.processEvents()
                    time.sleep(0.2)
                if (self.interrupted):
                    self.solveStatus.setText("Interrupted")
                    self.solveStatus.setStyleSheet("background-color: red")
                else:
                    self.solveStatus.setText("Done")
                    self.solveStatus.setStyleSheet("background-color: green")
                self.update()

    def solver (self, tab):
        tab.plugin.basicClient.problem.solve()

    def interruptPathPlanning(self):
        self.plugin.basicClient.problem.interruptPathPlanning()
        self.interrupted = True


#------------------------------------------------------------------end _RbprmInterp
#------------------------------------------------------------------start plugin:

class Plugin(QtGui.QDockWidget):
    """
    This plugin interacts with PythonWidget C++ class.
    """
    def __init__ (self, mainWindow, flags = None):
        if flags is None:
            super(Plugin, self).__init__ ("Rbprm plugin", mainWindow)
        else:
            super(Plugin, self).__init__ ("Rbprm plugin", mainWindow, flags)
        self.viewCreated = False
        self.client = Client ()
        self.rbprmClient = rbprmClient ()
        self.builder = Builder ()
        self.fullbody = FullBody ()
        self.basicClient  = basicClient ()
        self.affClient = affClient ()
        self.cursor = QtGui.QCursor ()
        #self.cursor.setShape (QtCore.Qt.OpenHandCursor)
        #self.cursor.setShape (QtCore.Qt.ClosedHandCursor)
        # add shortcut:
        action = self.toggleViewAction()
        action.setShortcut(QtGui.QKeySequence("Ctrl+Alt+A"))
        #initialise tab widget
        self.tabWidget = QtGui.QTabWidget()
        self.setWidget (self.tabWidget)
        self.rbprmPath = _RbprmPath (self,self)
        self.tabWidget.addTab (self.rbprmPath, "Tab 1")
        self.rbprmInterp = _RbprmInterp(self, self)
        self.tabWidget.addTab (self.rbprmInterp, "Tab 2")
        self.main = mainWindow
        mainSize = self.main.size
        self.tabWidget.setMaximumSize(int(float(mainSize.width())*0.6), mainSize.height())
        mainWindow.connect('refresh()', self.refresh)
        self.chooseWithMouse = False
        self.tabWidget.connect(self.tabWidget, QtCore.SIGNAL("currentChanged(int)"), self.updateSelected)

    ### If present, this function is called when a new OSG Widget is created.
    def osgWidget(self, osgWindow):
        osgWindow.connect('selected(QString,QVector3D)', self.selected)
        osgWindow.setMouseTracking(True)
        self.osg = osgWindow
        self.osg.installEventFilter(self)
        self.osg.acceptDrops = True
        #label = QtGui.QLabel("t")
        #label.mouseMoveEvent = self.mouseMoveEvent
        #osgWindow.mouseMoveEvent = self.mouseMoveEvent #(QtCore.QEvent.MouseMove)',self.mouseMove)
        
        #osgWindow.cursor.pos()connect('')

    def resetConnection(self):
        self.client = Client()
        self.rbprmClient = rbprmClient ()
        self.basicClient = basicClient ()
        self.affClient = affClient ()

    def updateSelected(self):
        currentWidget=self.tabWidget.currentWidget()
        currentWidget.update()
        self.rbprmInterp.resetStatus()

    def refresh(self):
        mainSize = self.main.size
        self.tabWidget.setMaximumSize(int(float(mainSize.width())*0.6), mainSize.height())
        self.rbprmInterp.update()
        self.rbprmPath.update()

    def mouseMoveEvent (self, mouseEvent):
        if (mouseEvent == QtCore.QEvent.MouseMove):
            print ("moved mouse")

    def eventFilter(self, source, event):
        if (event.type() == QtCore.QEvent.MouseMove):# and
            #source is self.osg):
            print('mouse at:', event.x(), event.y())
        #    print ("osg mouse:", source.)
            return False
        return QtGui.QWidget.eventFilter(self, source, event)

    def selected(self, name, posInWorldFrame):
        if (self.chooseWithMouse):
         #   QtGui.QMessageBox.information(self, "Selected object", name + " " + str(posInWorldFrame))
            #if str(name).find(self.builder.name) > -1:
            #    self.osg.cursor.setShape(QtCore.Qt.ClosedHandCursor)
            self.rbprmPath.spinConfig(self.rbprmPath.qvector2float (posInWorldFrame))
#----------------------------------------------------------------------------------------- end plugin
#----------------------------------------------------------------------------------------- start settings
class Settings ():
    def __init__(self):
        path = ""
        pathname = 'CMAKE_PREFIX_PATH'
        env = QtCore.QProcessEnvironment.systemEnvironment()
        if (env.contains(pathname)): 
            paths = (str(env.value(pathname))).split(':')
            for p in paths:
               directory = QtCore.QDir(p + '/etc')
               if directory.exists():
                  path = directory.path() 
                  break
        QtCore.QSettings.setPath (QtCore.QSettings.IniFormat, QtCore.QSettings.SystemScope, str(path))
        QtCore.QSettings.setPath (QtCore.QSettings.NativeFormat, QtCore.QSettings.SystemScope, str(path))
        self.robots = []
        self.envs = []
        self.fullbodies = []

    def readRobots(self):
        # second argument is directory name as follows: path/[gepetto-gui]/[rbprmRobots].conf
        robot = QtCore.QSettings (QtCore.QSettings.SystemScope,"gepetto-gui", "rbprmRoms")
        #robot.setValue("test", "value");
        #keys = robot.allKeys()
        #print (keys)
        if (robot.status () != QtCore.QSettings.NoError):
            print ("Error occurred when opening rbprm-robot config file.")
            return
        else:
            for child in robot.childGroups():
                info = []
                info.append (str(robot.value("RobotName", child)))
                info.append (str(robot.value("ModelName", "")))
                urdfROMs = str(robot.value("URDFRomNames", ""))
                urdfROMs = urdfROMs.replace(" ", "")
                info.append (urdfROMs.split(','))
                info.append (str(robot.value("RootJointType", "freeflyer")))
                info.append (str(robot.value("MeshPackage", "")))
                info.append (str(robot.value("Package", "")))
                info.append (str(robot.value("URDFSuffix", "")))
                info.append (str(robot.value("SRDFSuffix", "")))
                self.robots.append(info)
                print ("going through elements", info)
                
    def readEnvironments(self):
        env = QtCore.QSettings (QtCore.QSettings.SystemScope,"gepetto-gui", "rbprmEnvironments")
        if (env.status () != QtCore.QSettings.NoError):
            print ("Error occurred when opening rbprm-environment config file.")
            return
        else:
            for child in env.childGroups():
                info = []
                info.append (str(env.value("Package","")))
                info.append (str(env.value("URDFFilename", "")))
                info.append (str(env.value("RobotName", child)))
                self.envs.append(info)

    def readFullbody (self):
        fb = QtCore.QSettings (QtCore.QSettings.SystemScope,"gepetto-gui", "rbprmFullbodies")
        if (fb.status () != QtCore.QSettings.NoError):
            print ("Error occurred when opening rbprm-environment config file.")
            return
        else:
            for child in fb.childGroups():
            #TODO: FINISH!
                info = []
                info.append (str(fb.value("Package","")))
                info.append (str(fb.value("URDFFilename", "")))
                info.append (str(fb.value("RobotName", child)))
                self.envs.append(info)




