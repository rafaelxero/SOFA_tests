import SofaRuntime
SofaRuntime.importPlugin("SofaComponentAll")

import Sofa.Core

import os
path = os.path.dirname(os.path.abspath(__file__))+'/design/'


def createScene(rootNode):
    
    rootNode.addObject("RequiredPlugin", pluginName = "SofaExporter")
    rootNode.addObject("RequiredPlugin", pluginName = "CGALPlugin")
    rootNode.addObject("RequiredPlugin", pluginName = "SofaOpenglVisual")

    rootNode.addObject("VisualStyle", displayFlags = "hideVisual")

    rootNode.addObject("MeshOBJLoader", name = "loader",
                       filename = path + "Tendon_Driven_Actuator.obj")

    rootNode.addObject("MechanicalObject", name = "dofs",
                       position = "@loader.position")

    rootNode.addObject("MeshGenerationFromPolyhedron", name = "gen",
                       inputPoints = "@loader.position",
                       inputTriangles = "@loader.triangles",
                       facetSize = "0.75", facetApproximation = "1",
                       cellRatio = "2", cellSize = "2",
                       drawTetras = "1")

    rootNode.addObject("MeshTopology", name = "topo",
                       position = "@gen.outputPoints",
                       tetrahedra = "@gen.outputTetras")

    rootNode.addObject("VTKExporter", filename = "finger",
                       src = "@topo", edges = "0", exportAtBegin = "1")
    
    rootNode.addObject("OglModel", color = [0.3, 0.2, 0.2, 0.6])
    
    return rootNode
