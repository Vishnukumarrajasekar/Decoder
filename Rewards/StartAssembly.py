import rhinoscriptsyntax as rs
import ghpythonlib.components as gh
import Rhino.Geometry as rg
import System.Drawing as sd
import math
import random
import Grasshopper as Grasshopper

import Assembly as assembly
import Grammar as gr
import Rhino as rc
import Rules as rules
import GraphicStatics as gs

reload (assembly)
reload (gs)
reload (rules)
reload (gr)



# node0 : Load Point 1
line = rg.Line(rg.Point3d(15,20,0), rg.Point3d(15,15,0))
forcelist=[]
force = assembly.Force(line, 2, -1)

forcelist.append(force)

node = assembly.Node(rg.Point3d(15,15,0), 2, forcelist)
#node = gs.EquilibrateNode(node)





# node1 : Load point 2
line1 = gh.Line(rg.Point3d(30,20,0), rg.Point3d(30,15,0))
forcelist1=[]
force1 = assembly.Force(line1, 2, -1)
forcelist1.append(force1)
node1 = assembly.Node(rg.Point3d(30,15,0), 2, forcelist1)
#node1 = gs.RearrangeForces(node1)
#node1 = gs.EquilibrateNode(node1)



# node2 : Reaction 1
line2 = gh.Line(rg.Point3d(-7,-5,0), rg.Point3d(0,0,0))
forcelist2=[]
force2 = assembly.Force(line2, 2, -1)
forcelist2.append(force2)
node2 = assembly.Node(rg.Point3d(0,0,0), 1, forcelist2)
#node2 = gs.RearrangeForces(node2)
#node2 = gs.EquilibrateNode(node2)

# node 3 : Reaction 2
line3 = gh.Line(rg.Point3d(52,-5,0), rg.Point3d(45,0,0))
forcelist3=[]
force3 = assembly.Force(line3, 2, -1)
forcelist3.append(force3)
node3 = assembly.Node(rg.Point3d(45,0,0), 1, forcelist3)
#node3 = gs.RearrangeForces(node3)
#node3 = gs.EquilibrateNode(node3)