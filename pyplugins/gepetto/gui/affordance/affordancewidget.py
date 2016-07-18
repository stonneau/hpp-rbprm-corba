from PythonQt import QtGui, Qt
from gepetto.corbaserver import Client
from hpp.corbaserver.rbprm import Client as rbprmClient
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
        box.addWidget(self.bindFunctionToButton("Add or Edit Affordance Configuration", self.setAffConfig))

        # Add to group
        self.groupNodes = QtGui.QComboBox(self)
        self.groupNodes.editable = False
        box.addWidget(self.addWidgetsInHBox( [
            self.groupNodes,
            self.bindFunctionToButton("Add to group", self.addToGroup)
                ]))

        # Affordance Analysis Button
        self.affAnalysisOptions = QtGui.QComboBox(self)
        self.affAnalysisOptions.editable = False
        box.addWidget(self.addWidgetsInHBox( [
            self.bindFunctionToButton("Find Affordances for", self.addToGroup),
            self.affAnalysisOptions
            ]))

        # Add mesh
        #box.addWidget(self.bindFunctionToButton("Add mesh", self.addMesh))

        # Add box
        #box.addWidget(self.bindFunctionToButton("Add box", self.addBox))

        self.update()

    def update(self):
        self.groupNodes.clear()
        for n in self.plugin.client.gui.getSceneList():
            self.groupNodes.addItem (n)
        self.affordanceTypes.clear()
        affordances = self.plugin.affClient.affordance.getAffordanceTypes()
        for aff in affordances:
            self.affordanceTypes.addItem (aff)
            self.affAnalysisOptions.addItem (aff)
        self.affAnalysisOptions.addItem("All")

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
        self.rbprmClient  = rbprmClient ()
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
        self.affClient = affClient ()

    def refresh(self):
        self.nodeCreator.update()

    def selected(self, name, posInWorldFrame):
        QtGui.QMessageBox.information(self, "Selected object", name + " " + str(posInWorldFrame))
