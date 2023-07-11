import SofaRuntime
SofaRuntime.importPlugin("SofaComponentAll")

import Sofa.Core


class SurfacePressureController(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):

        print("init")
        
        Sofa.Core.Controller.__init__(self, *args, **kwargs)

        self.node = kwargs['node']
        self.volume = kwargs['volume']
        self.pressure = kwargs['pressure']
        self.time = self.node.dt.value

    def onAnimateEndEvent(self, event):

        print("onAnimatedEndEvent")

        self.time = self.time + self.node.dt.value

        volume_des = 80

        print(self.volume.value)
        
        #volume = self.volume.value[0] + 1
        pressure = 0.65 * (volume_des - self.volume.value)

        #if volume >= 200:
        #    volume = 200
            
        #self.volume.value[0] = volume
        self.pressure.value[0] = pressure

        #print(self.volume.value[0])
        print(self.pressure.value[0])
        

def createScene(rootNode):

    rootNode.addObject("RequiredPlugin", pluginName = "SofaSparseSolver")

    rootNode.addObject("VisualStyle",
                       displayFlags = "showCollision showVisualModels showForceFields showInteractionForceFields hideCollisionModels hideBoundingCollisionModels hideWireframe")
    
    rootNode.dt = 0.01
    rootNode.name = "rootNode"

    rootNode.addObject("FreeMotionMasterSolver")
    rootNode.addObject("GenericConstraintSolver",
                       name = "GenericConstraintSolver",
                       maxIterations = 250, printLog = 0, tolerance = 0.0000001)

    rootNode.addObject("CollisionPipeline", verbose = 0)
    rootNode.addObject("BruteForceBroadPhase", name = "N2")
    rootNode.addObject("BVHNarrowPhase")
    rootNode.addObject("CollisionResponse", response = "FrictionContact")
    rootNode.addObject("LocalMinDistance", name = "Proximity",
                       alarmDistance = 3, contactDistance = 0.5)

    bunny = rootNode.addChild("bunny")

    bunny.addObject("EulerImplicitSolver", firstOrder = 0)
    bunny.addObject("SparseLDLSolver")

    bunny.addObject("MeshVTKLoader", name = "loader",
                    filename = "mesh/Hollow_Stanford_Bunny.vtu")

    bunny.addObject("TetrahedronSetTopologyContainer",
                    src = "@loader")

    bunny.addObject("MechanicalObject",
                    name = "tetras", template = "Vec3d")

    bunny.addObject("UniformMass", totalMass = 0.5)
    
    bunny.addObject("TetrahedronFEMForceField",
                    youngModulus = 18000, poissonRatio = 0.3,
                    method = "large")

    bunny.addObject("BoxROI", name = "ROI1",
                    box = [-5, -15, -5, 5, -4.5, 5], drawBoxes = True)
    bunny.addObject("RestShapeSpringsForceField",
                    points = "@ROI1.indices", stiffness = 1e12)
    
    bunny.addObject("LinearSolverConstraintCorrection")
    
    cavity = bunny.addChild("cavity")

    cavity.addObject("MeshObjLoader", name = "loader",
                     filename = "mesh/Hollow_Bunny_Body_Cavity.obj")

    cavity.addObject("Mesh", name = "topo", src = "@loader")

    cavity.addObject("MechanicalObject", name = "cavity")

    cavity.addObject("SurfacePressureConstraint", name = "SurfacePressureConstraint",
                     value = 0, valueType = "pressure")
                     #value = 40, valueType = "pressure")

    cavity.addObject("VolumeFromTriangles", name = "VolumeFromTriangles")

    cavity.addObject("BarycentricMapping", mapForces = False, mapMasses = False)

    print("about to attach controller")
    
    cavity.addObject(SurfacePressureController(node = rootNode,
                                               volume = rootNode.bunny.cavity.SurfacePressureConstraint.cavityVolume,
                                               pressure = rootNode.bunny.cavity.SurfacePressureConstraint))
                                               #volume = rootNode.bunny.cavity.SurfacePressureConstraint))
    
    return rootNode
