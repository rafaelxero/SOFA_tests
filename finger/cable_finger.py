import SofaRuntime
SofaRuntime.importPlugin("SofaComponentAll")

import Sofa.Core

import math
import numpy as np
from scipy import signal

import os
path = os.path.dirname(os.path.abspath(__file__))+'/design/'


class FingerController(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):

        Sofa.Core.Controller.__init__(self, *args, **kwargs)

        self.node = kwargs['node']
        self.displ = kwargs['displacement']
        self.Positions = kwargs['position']
        self.time = self.node.dt.value

        # base search

        self.minimum = 0
        self.posmin = 0

        for i in range(0, len(self.Positions)):
            if self.Positions[i][2] <= self.minimum and self.Positions[i][0] >= -0.1 and self.Positions[i][0] <= 0.1:
                self.minimum = self.Positions[i][2]
                self.posmin = i

        # fingertip search

        self.max = 0
        self.posmax = 0

        for i in range(0, len(self.Positions)):
            if self.Positions[i][2] >= self.max and self.Positions[i][0] >= -0.1 and self.Positions[i][0] <= 0.1:
                self.max = self.Positions[i][2]
                self.posmax = i

        self.coeff = (self.Positions[self.posmax][2] - self.Positions[self.posmin][2]) / (self.Positions[self.posmax][0] - self.Positions[self.posmin][0])

        self.angle = ((math.atan(self.coeff)) * 180 / math.pi) - 90
        print(self.angle)

        file1 = open("/home/rcisneros/devel/rcisneros/tests/tutorials/SOFA/finger/plot/Position_endEffector.txt", "w")
        file1.write(str(0.0) + ' ' + str(self.Positions[self.posmax][0]) + ' ' + str(self.Positions[self.posmax][1]) + ' ' + str(self.Positions[self.posmax][2]) + '\n')
        file1.close()

        file2 = open("/home/rcisneros/devel/rcisneros/tests/tutorials/SOFA/finger/plot/Length-Angle.txt", "w")
        file2.write(str(0.0) + ' ' + str(0.0) + ' ' + str(self.angle) + '\n')
        file2.close()

        
    def onAnimateEndEvent(self, event):
    
        self.time = self.time + self.node.dt.value

        # displ = self.displ.value[0] + 0.9

        # if displ >= 6:
        #     displ = 6

        displ = -3 * np.cos(2 * math.pi * 0.5 * self.time) + 3
        
        self.displ.value[0] = displ

        coeff = (self.Positions[self.posmax][2] - self.Positions[self.posmin][2]) / (self.Positions[self.posmax][0] - self.Positions[self.posmin][0])

        angle = ((math.atan(coeff)) * 180 / math.pi) - 90
    
        file1 = open("/home/rcisneros/devel/rcisneros/tests/tutorials/SOFA/finger/plot/Position_endEffector.txt", "a")
        file1.write(str(self.time) + ' ' + str(self.Positions[self.posmax][0]) + ' ' + str(self.Positions[self.posmax][1]) + ' ' + str(self.Positions[self.posmax][2]) + '\n')
        file1.close()
        
        file2 = open("/home/rcisneros/devel/rcisneros/tests/tutorials/SOFA/finger/plot/Length-Angle.txt", "a")
        file2.write(str(self.time) + ' ' + str(displ) + ' ' + str(angle) + '\n')
        file2.close()

            
def createScene(rootNode):
    
    rootNode.addObject("RequiredPlugin", pluginName = "SofaPython3")
    rootNode.addObject("RequiredPlugin", pluginName = "SoftRobots")
    rootNode.addObject("RequiredPlugin", pluginName = "SofaOpenglVisual")
    rootNode.addObject("RequiredPlugin", pluginName = "SofaSparseSolver")
    rootNode.addObject("RequiredPlugin", pluginName = "SofaPreconditioner")
    rootNode.addObject("RequiredPlugin", pluginName = "SofaMiscCollision")

    rootNode.addObject("VisualStyle",
                       displayFlags = "showVisualModels showInteractionForceFields")

    rootNode.gravity = [0, 0, 9810]
    rootNode.dt = 0.01
    rootNode.name = "rootNode"

    rootNode.addObject("FreeMotionAnimationLoop")
    rootNode.addObject("GenericConstraintSolver",
                       name = "consolver", tolerance = 1e-7,
                       maxIterations = 10000, computeConstraintForces = True)

    rootNode.addObject("DefaultPipeline")
    rootNode.addObject("BruteForceBroadPhase")
    rootNode.addObject("BVHNarrowPhase")
    rootNode.addObject("DefaultContactManager",
                       response = "FrictionContact",
                       responseParams = "mu = 0.05")
    rootNode.addObject("LocalMinDistance", name = "Proximity",
                       alarmDistance = 0.5, contactDistance = 0.1, angleCone = 0.0)

    rootNode.addObject("BackgroundSetting", color = [0, 0.168627, 0.211765])
    rootNode.addObject("OglSceneFrame", style = "Arrows", alignment = "TopRight")

    # finger

    finger = rootNode.addChild("finger")

    finger.addObject("EulerImplicitSolver", name = "odesolver",
                     rayleighStiffness = 0.1, rayleighMass = 0.1)
    finger.addObject("SparseLDLSolver", name = "directSolver")

    finger.addObject("MeshVTKLoader", name = "loader",
                     filename = path + "finger0.vtu")

    finger.addObject("MechanicalObject",
                     name = "tetras", template = "Vec3d",
                     src = "@loader", position = "@loader.position")

    finger.addObject("TetrahedronSetTopologyContainer",
                     name = "topo", src = "@loader")
    finger.addObject("TetrahedronSetTopologyModifier",
                     name = "Modifier", listening = True)
    finger.addObject("TetrahedronSetGeometryAlgorithms",
                     template = "Vec3d", name = "GeomAlgo", listening = True)

    finger.addObject("MeshMatrixMass",
                     massDensity = "1.020e-9", src = "@topo")

    mu1 = 2*109
    k0 = 2/1.150e-3

    finger.addObject("TetrahedronHyperelasticityFEMForceField",
                     template = "Vec3d", name = "FEM", src = "@topo",
                     ParameterSet = str(mu1) + ' ' + str(k0), materialName = "NeoHookean")

    finger.addObject("BoxROI", name = "boxROI",
                     box = [-12, -5, -70, 10, 15, -90], drawBoxes = True)
    finger.addObject("RestShapeSpringsForceField", points = "@boxROI.indices",
                     stiffness = 1e12, angularStiffness = 1e12)
    finger.addObject("LinearSolverConstraintCorrection",
                     solverName = "directSolver", listening = True)

    # cable

    cable = finger.addChild("cable")

    cable.addObject("EulerImplicitSolver", name = "odesolver",
                     rayleighStiffness = 0.1, rayleighMass = 0.1)
    cable.addObject("SparseLDLSolver", name = "directSolver")

    pullPointLocation = [-10.0, 5.0, -100]
    initialValue = 0.0
    valueType = "displacement"

    cable.addObject("MechanicalObject", name = "mech",
                    position = '-10 5 -73.113' + '\n' + '-10 5 -61.387' +
                    '\n' + '-10 5.5 -53.613' + '\n' + '-10 5 -41.887' +
                    '\n' + '-10 5 -34.113' + '\n' + '-10 5 -22.387' +
                    '\n' + '-10 5 -14.611' + '\n' +'-10 5 -2.885',
                    showObject = True, showObjectScale = 10)

    cable.addObject("CableConstraint", indices = [0, 1, 2, 3, 4, 5, 6, 7],
                    pullPoint = pullPointLocation, value = initialValue,
                    valueType = valueType, hasPullPoint = True,
                    listening = True, group = 1)

    cable.addObject("BarycentricMapping", name = "Mapping",
                    mapForces = False, mapMasses = False)

    cable.addObject(FingerController(node = rootNode,
                                     position = rootNode.finger.tetras.position.value,
                                     displacement = rootNode.finger.cable.CableConstraint))

    # collision model - finger

    collisionFinger = finger.addChild("collisionFinger")

    collisionFinger.addObject("MeshObjLoader", name = "loader",
                              filename = path + "Tendon_Driven_Actuator.obj")

    collisionFinger.addObject("MeshTopology", src = "@loader", name = "topo")

    collisionFinger.addObject("MechanicalObject", name = "collisMech",
                              position = "@loader.position")

    collisionFinger.addObject("TriangleCollisionModel", selfCollision = False)
    collisionFinger.addObject("LineCollisionModel", selfCollision = False)
    collisionFinger.addObject("PointCollisionModel", selfCollision = False)

    collisionFinger.addObject("BarycentricMapping")
    
    # visual model - finger
    
    modelVisu1 = finger.addChild("visu")
    
    modelVisu1.addObject("MeshObjLoader", name = "loader",
                         filename = path + "Tendon_Driven_Actuator.obj")

    modelVisu1.addObject("OglModel", src = "@loader", color = [1, 0.64, 0, 100])

    modelVisu1.addObject("BarycentricMapping")

    # floor

    planeNode = rootNode.addChild("Plane")

    planeNode.addObject("MeshObjLoader", name = "loader",
                        filename = path + "floorFlat.obj",
                        rotation = [-90, 0, 0],
                        translation = [-44, 0, 5],
                        scale = 5)

    planeNode.addObject("MeshTopology", src = "@loader")
    
    planeNode.addObject("MechanicalObject", src = "@loader")

    planeNode.addObject("TriangleCollisionModel")
    planeNode.addObject("LineCollisionModel")
    planeNode.addObject("PointCollisionModel")

    planeNode.addObject("OglModel", name = "Visual",
                        src = "@loader", color = [1, 0, 0, 1])

    # cube

    cube = rootNode.addChild("cube")

    cube.addObject("EulerImplicitSolver", name = "odesolver",
                   rayleighStiffness = 0.1, rayleighMass = 0.1)
    cube.addObject("SparseLDLSolver", name = "directSolver")

    cube.addObject("MechanicalObject", name = "collisMech",
                   template = "Rigid3", position = [-25, 2.5, -3, 0, 0, 0, 1])

    cube.addObject("UniformMass", totalMass = 0.001)

    cube.addObject("UncoupledConstraintCorrection")

    # collision model - cube

    cubecollis = cube.addChild("cubecollis")

    cubecollis.addObject("MeshObjLoader", name = "loader",
                         filename = path + "smCube27.obj", scale = 2)

    cubecollis.addObject("MeshTopology", src = "@loader", name = "topo")

    cubecollis.addObject("MechanicalObject")

    cubecollis.addObject("TriangleCollisionModel")
    cubecollis.addObject("LineCollisionModel")
    cubecollis.addObject("PointCollisionModel")

    cubecollis.addObject("RigidMapping")

    # visual model - cube

    modelVisu = cube.addChild("visu")

    modelVisu.addObject("MeshObjLoader", name = "loader",
                        filename = path + "smCube27.obj")

    modelVisu.addObject("OglModel", src = "@loader",
                        color = "blue", scale = 2)

    modelVisu.addObject("RigidMapping")
    
    return rootNode
