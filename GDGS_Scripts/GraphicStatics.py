# Compas imports
from compas.geometry import Point, Vector, Line, Circle, Polyline, Polygon, Pointcloud, Plane, Frame, Translation, Scale, Rotation, intersection_line_line
from compas_occ.geometry import OCCNurbsCurve
import compas.geometry as cg

from compas_plotters import Plotter


from compas_view2.app import App

import math

################################################################################

import Assembly as assembly
import RandomUtility as ru

################################################################################
# Custom Functions
################################################################################

#sort points 
def SortPoints(points):
    sorted_points = sorted(points, key=lambda pt: (pt.x, pt.y, pt.z))
    return sorted_points

# Amplitude from vector or line 
def Amplitude(input, length):
    if isinstance(input, Vector):
        unit_vec = input.unitized()
        scaled_vec = unit_vec.scaled(length)
        return scaled_vec

    elif isinstance(input, Line):
        vector = Vector.from_start_end(input.start, input.end)
        unit_vec = vector.unitized()
        scaled_vec = unit_vec.scaled(length)
        return scaled_vec

    else:
        return (type(input))

""" # Amplitude from vector 
def Amplitude(vector, length):
    unit_vec = vector.unitized()
    scaled_vec = unit_vec.scaled(length)
    return scaled_vec

# Amplitude from line 
def Amplitude(line, length):
    vector = Vector.from_start_end(line.start, line.end)
    unit_vec = vector.unitized()
    scaled_vec = unit_vec.scaled(length)
    return scaled_vec """

# Move point or line 
def Move(input, vector):
    T = Translation.from_vector(vector)
    if isinstance(input, Point):
        point1 = Point.copy(input)
        point1.transform(T)
        return point1
    elif isinstance(input, Line):
        line1 = Line.copy(input)
        line1.transform(T)
        return line1
    elif isinstance(input, Vector):
        vector1 = Vector.copy(input)
        vector1.transform(T)
        return vector1
    else:
        return type(input)

""" def Move(point, vector):
    T = Translation.from_vector(vector)
    point1 = Point.copy(point)
    point1.transform(T)
    return point1

def Move(line, vector):
    T  = Translation.from_vector(vector)
    line1 = Line.copy(line)
    line1.transform(T)
    return line1 """

def LineSDL(point,vector,length):
    amp = Amplitude(vector,length)
    moved_point = Move(point, amp)
    line = Line(point, moved_point)   
    return line

def UnitZ(point):
    dup_vec = Vector.copy(point)
    dup_vec.z = 1
    return dup_vec

def Rotate(line, angle, zaxis):
    R1 = Rotation.from_axis_and_angle(zaxis, angle)
    line_copy = Line.copy(line)
    line_copy.transform(R1)
    return line_copy

# Closest point from a point cloud 
def ClosestPoint (org, points):
    sorted_points = sorted(points, key=lambda point: (org.distance_to_point(point), points.index(point)))

    return sorted_points[0]

# Closest point from a point cloud 
def ClosestPoints (point, points, count=None):
    
    sorted_points = sorted(points, key=lambda p: (p.distance_to_point(point)))

    sorted_indices = [points.index(p) for p in sorted_points]
    
    if count is None:
        # If 'count' is not specified, return all points (no limit)
        return sorted_points, sorted_indices
    else:
        # Return the specified number of closest points
        return sorted_points[:count], sorted_indices[:count]

# CurveXCurves 
def CurveXCurves (line, lines):
    points = []
    for l in lines:
            point = intersection_line_line(line, l)[0]
            points.append(point)
    return points

################################################################################
# FORCE OPERATIONS
################################################################################

def TranslateForce (force, coordinate):
    
    # PURPOSE : Move force to a new coordinate
    
    # INPUT : 1) assembly.force instance  2) assembly.node instance
    # OUTPUT : assembly.force instance
    
    #if compression
    if force.Direction == -1:
        """ start = gh.EndPoints(force.Line)[1] #get ending point
        finish = coordinate
        translation = gh.Vector2Pt(start, finish)[0]
        movedline = gh.Move(force.Line, translation)[0]
        force.Line = movedline
        return force """
        translation_vec = Vector.from_start_end(force.Line.end, coordinate)
        T = Translation.from_vector(translation_vec)
        force.Line.transform(T)
        return force
        

    
    #if tension
    else:
        """ start = gh.EndPoints(force.Line)[0] #get starting point
        finish = coordinate
        translation = gh.Vector2Pt(start, finish)[0]
        movedline = gh.Move(force.Line, translation)[0]
        force.Line = movedline
        return force """
        translation_vec = Vector.from_start_end(force.Line.start, coordinate)
        T = Translation.from_vector(translation_vec)
        force.Line.transform(T)
        return force

def ReverseForce (force):
    
    # PURPOSE : Reverse direction of force 
    
    # INPUT : assembly.force instance  
    # OUTPUT : assembly.force instance

    """ start = gh.EndPoints(force.Line)[0]
    end = gh.EndPoints(force.Line)[1]
    force.Line = gh.Line(end, start)
    force.Direction = force.Direction*-1
    return force """
    force.Line = Line(force.Line.start, force.Line.end)
    force.Direction = force.Direction*-1
    return force

def GetTempForce (node):
    
    # PURPOSE : Identifies and returns the temp force at a node. 
    
    # INPUT : assembly.node instance
    # OUTPUT : list of temporaryasssembly.force instance.
    
    # Get Temp Force at Node
    TempForces = []
    for i in range(0, len(node.Forces)):
        if node.Forces[i].Type == 3: #if the force is a temporary force
            TempForces.append(node.Forces[i])
    return TempForces

def FlipTempForceDir (node):
    #flip direction  
    NodeForces = []
    for i in range(0, len(node.Forces)):
        if node.Forces[i].Type != 3: #if the force is not a temporary force
            NodeForces.append(node.Forces[i])
    TempForce = GetTempForce (node)[0]
    TempForce.Direction = TempForce.Direction*-1
    NodeForces.append(TempForce)
    node.Forces = NodeForces
    node = RearrangeForces(node)
    return node

def GetLineOfAction (node):
    
    # PURPOSE : Create line of action vector. 
    
    # INPUT : assembly.node instance
    # OUTPUT : list of unit vectors, aligned to temp force away from the node.
    
    # Get Temp Force at Node
    TempForces = GetTempForce(node)
    
    # Determine Line of Action
    LOA = []
    for force in TempForces:
        """ circle = gh.Circle(node.Coordinate, 0.1)
        circle = gh.Scale(circle, node.Coordinate, 0.1)[0]
        intersection = gh.CurveXCurve(force.Line, circle)
        LOA.append(gh.Vector2Pt(node.Coordinate, intersection)[0]) """
        if force.Line.start == node.Coordinate:
            vec = Vector.copy(force.Line.direction)
        else:
            vec = Vector.copy(force.Line.direction)
            vec *= -1
        LOA.append(vec)
    return LOA

def LineToVector (line):
    """ startpoint = gh.EndPoints(line)[0]
    endpoint = gh.EndPoints(line)[1]
    vector = gh.Vector2Pt(startpoint, endpoint) """
    
    return line.vector

def TransferMemberForce (force, nodecoordinate):
    
    # PURPOSE : Move force instance to a point. 
    
    # INPUT : 1) assembly.force instance   2) assembly.node instance
    # OUTPUT : assembly.force instance
    
    InitialForceLine = force.Line
    
    """ ForceLineStart = gh.EndPoints(InitialForceLine)[0]
    ForceLineEnd = gh.EndPoints(InitialForceLine)[1] """
    ForceLineStart = InitialForceLine.start
    ForceLineEnd = InitialForceLine.end

    if force.Direction == -1: #if member is in compression
        """ translation = gh.Vector2Pt(ForceLineStart, nodecoordinate)[0] """
        translation_vec = Vector.from_start_end(ForceLineStart, nodecoordinate)
        
    else:
        """ translation = gh.Vector2Pt(ForceLineEnd, nodecoordinate)[0] """
        translation_vec = Vector.from_start_end(ForceLineEnd, nodecoordinate)
    
    """ TransferedForceLine = gh.Move(InitialForceLine, translation)[0]
    TransferedForceLine = gh.FlipCurve(TransferedForceLine)[0] """
    
    T = Translation.from_vector(translation_vec)
    TransferedForceLine = InitialForceLine.transform(T)
    TransferedForceLine = Line(TransferedForceLine.end, TransferedForceLine.start)
    
    newforce = assembly.Force(TransferedForceLine, force.Type, force.Direction)
    return newforce

def GetPosWayRay (node):
    vector = GetLineOfAction(node)
    point = node.Coordinate
    """ ray = gh.LineSDL(point, vector, 1000) """

    ray = LineSDL(point, vector, 1000)
    return ray

def GetNegWayRay (node):
    vector = GetLineOfAction(node)
    point = node.Coordinate
    """ ray = gh.LineSDL(point, vector, -1000) """

    ray = LineSDL(point, vector, -1000)
    return ray

def GetTwoWayRay (node):
    vector = GetLineOfAction(node)
    point = node.Coordinate
    """ move = gh.Amplitude(vector, 1000)
    point1 = gh.Move(point, move)[0]
    ray = gh.LineSDL(point1, vector, -2000) """
    
    """ unit_vec = vector.unitized()
    move_vec = unit_vec.scaled(1000)
    T = Translation.from_vector(move_vec)
    point1 = Point.copy(point)
    point1.transform(T)
    ray = LineSDL(point1, vector, -2000) """

    move = Amplitude(vector, 1000)
    point1 = Move(point, move)
    ray = LineSDL(point1, vector, -2000)

    return ray

def UniDirVectorIntersection (point1, vector1, point2, vector2):
    """ ray1 = gh.LineSDL(point1, vector1, 1000)

    ray2 = gh.LineSDL(point2, vector2, 1000)
    
    intersection = gh.CurveXCurve(ray1, ray2)[0] """

    ray1 = LineSDL(point1, vector1, 1000)
    ray2 = LineSDL(point2, vector2, 1000)
    intersection = intersection_line_line(ray1, ray2)[0]

    return intersection

def BiDirVectorIntersection (point1, vector1, point2, vector2):
    """ move1 = gh.Amplitude(vector1, 1000)
    point1 = gh.Move(point1, move1)[0]
    ray1 = gh.LineSDL(point1, vector1, -2000)
    
    move2 = gh.Amplitude(vector2, 1000)
    point2 = gh.Move(point2, move2)[0]
    ray2 = gh.LineSDL(point2, vector2, -2000)
    
    intersection = gh.CurveXCurve(ray1, ray2)[0] """

    move1 = Amplitude(vector1, 1000)
    point1 = Move(point1, move1)[0]
    ray1 = LineSDL(point1, vector1, -2000)

    move2 = Amplitude(vector2, 1000)
    point2 = Move(point2, move2)[0]
    ray2 = LineSDL(point2, vector2, -2000)

    intersection = intersection_line_line(ray1, ray2)[0]
    return intersection


################################################################################
# NODE OPERATIONS

def GetActiveNodes (assembly):
    activenodes = []
    for allnodes in assembly.Nodes:
        if allnodes.NodeState == 1:
            activenodes.append(allnodes)
    return activenodes

def RearrangeForces (node): 
    
    # PURPOSE : Rearrange Forces, clockwise starting at x axis
    
    # INPUT : assembly.node instance
    # OUTPUT : assembly.node instance
    
    #verify locations
    forcelist = node.Forces
    for force in forcelist:
        force = TranslateForce(force, node.Coordinate)
    
    """ #rearrange
    Intersections = []
    circle = gh.Circle(node.Coordinate, 0.1)
    circle = gh.Scale(circle, node.Coordinate, 0.1)[0]
    circle2 = gh.FlipCurve(circle)[0] # to sort clockwise 
    
    if len(forcelist) == 1:
        return node

    else:
        for i in range(0, len(forcelist)):
            line = forcelist[i].Line
            point = gh.CurveXCurve(line, circle2)[0]
            Intersections.append(point)
        
        SortedPointList = gh.SortAlongCurve(Intersections, circle2)
        SortOrder = SortedPointList[1]
        SortedList = [forcelist[j] for j in SortOrder]
        node.Forcelist = SortedList #reassemble node forcelist
        return node """

       
    #rearrange
    Intersections = []
    plane = Plane(node.Coordinate, [0,0,1])
    circle = Circle(plane, 0.1)
    circle.radius = circle.radius * 0.1
    crv = OCCNurbsCurve.from_circle(circle)

    if len(forcelist) == 1:
        return node
    
    else:
        far_points = []
        for i in range(0, len(forcelist)):
            line = forcelist[i].Line
            if line.start == node.Coordinate: 
                far_points.append(line.end)
            else:
                far_points.append(line.start)

        parameters = []
        for pt in far_points:
            closest, t = crv.closest_point(pt, return_parameter=True)
            parameters.append(t)
        
        SortedList = [point for _, point in sorted(zip(parameters, node.Forces))]
        node.Forces = SortedList
        return node
    
def TempNodeEquilibrium (floatnode):
    
    # PURPOSE : Equilibrate a float node with 3, unit-tempforces . 
    
    # INPUT : assembly.node float instance (with empty list of forces)
    # OUTPUT : assembly.node float instance (with forces)
    
    """ origin = rg.Point3d(0,0,0)
    newpoint = rg.Point3d(0,-1,0) """

    origin = Point(0,0,0)
    newpoint = Point(0,-1,0)
  
    """ forceline1 = gh.Line(origin, newpoint)
    forceline2 = gh.Rotate(forceline1, 120*(math.pi/180), origin)[0]
    forceline3 = gh.Rotate(forceline1, 240*(math.pi/180), origin)[0] """
    

    forceline1 = Line(origin, newpoint)
    forceline2 = Rotate(forceline1, radians(120), UnitZ(origin))
    forceline3 = Rotate(forceline1, radians(240), UnitZ(origin))



    forcelist = []
    force1 = assembly.Force(forceline1, 3, -1)
    force2 = assembly.Force(forceline2, 3, -1)
    force3 = assembly.Force(forceline3, 3, -1)
    forcelist.append(force1)
    forcelist.append(force2)
    forcelist.append(force3)
    
    floatnode.Forces = forcelist
    floatnode = RearrangeForces(floatnode)
    return floatnode

def EquilibrateNode (node): 
    
    # PURPOSE : Check and complete equilibrium of a node. 
    
    # INPUT : assembly.node instance
    # OUTPUT : assembly.node instance
    
    # 1. Verify Force Arrangement
    forcelist = node.Forces
    
    # 2. Generate force polygon
    polygon = []
    
    if len(forcelist) == 1: #if there is only single force on it
        polygon.append(forcelist[0].Line)

    else: #for every other condition
        node = RearrangeForces (node)
        polygon.append(forcelist[0].Line)
        for i in range(1,len(forcelist)):
            
            line1 = polygon[-1]
            """ startpoint1 = gh.EndPoints(line1)[0]
            endpoint1 = gh.EndPoints(line1)[1] """
            startpoint1 = line1.start
            endpoint1 = line1.end
            
            line2 = forcelist[i].Line
            """ startpoint2 = gh.EndPoints(line2)[0]
            endpoint2 = gh.EndPoints(line2)[1] """
            startpoint2 = line2.start
            endpoint2 = line2.end
            
            """ translation = gh.Vector2Pt(startpoint2, endpoint1)
            movedline = gh.Move(line2, translation)[0] """
            translation = Vector(startpoint2, endpoint1)
            movedline = Move(line2,translation)
            polygon.append(movedline)
    
    """ polygonjoined = gh.JoinCurves(polygon)
    polygonstart = gh.EndPoints(polygonjoined)[0]
    polygonend = gh.EndPoints(polygonjoined)[1] """
    pts = []
    for line in polygon:
        pts.append(line.start)
    pts.append(polygon[-1].end)
    polygonjoined = Polyline(pts)
    polygonstart = polygonjoined.points[0]
    polygonend = polygonjoined.points[-1]
    
    """ closingline = gh.Line(polygonend, polygonstart)
    polygon.append(closingline) """

    closingline = Line(polygonend, polygonstart)
    polygon.append(closingline) 

    
    """ center = gh.EndPoints(forcelist[0].Line)[1] #endpoint of any force line should be at the center
    translation = gh.Vector2Pt(polygonstart, center)
    movedline = gh.Move(closingline, translation)[0] """

    center = forcelist[0].Line.start #endpoint of any force line should be at the center
    translation = Vector.from_start_end(polygonstart, center)
    movedline = Move(closingline, translation)
    
    TempForce = assembly.Force(movedline, assembly.ForceType.Temporary, assembly.ForceDirection.Compression)
    
    forcelist.append(TempForce)

    # 3. Verify Force Arrangement
    node = assembly.Node(node.Coordinate, node.Type, forcelist) #reassemble node
    node = RearrangeForces (node)
    return node

def DrawNodeForcePolygon (node, drawlocation):

    forcelist = node.Forces
    
    polygon = []
    copyoffirstforce = assembly.Force(forcelist[0].Line, forcelist[0].Type, forcelist[0].Direction)
    polygon.append(copyoffirstforce)
    
    for i in range(1,len(forcelist)):
        #draws forcepolygon at node
        line1 = polygon[-1].Line
        """ startpoint1 = gh.EndPoints(line1)[0]
        endpoint1 = gh.EndPoints(line1)[1] """
        startpoint1 = line1.start
        endpoint1 = line1.end
        
        line2 = forcelist[i].Line
        """ startpoint2 = gh.EndPoints(line2)[0]
        endpoint2 = gh.EndPoints(line2)[1] """
        startpoint2 = line2.start
        endpoint2 = line2.end
        
        """ translation = gh.Vector2Pt(startpoint2, endpoint1)[0]
        movedline1 = gh.Move(line1, translation)[0]
        movedline2 = gh.Move(line2, translation)[0] """
        translation = Vector.from_start_end(startpoint2, endpoint1)
        movedline1 = Move(line1, translation)
        movedline2 = Move(line2, translation)
        
        movedforce2 = assembly.Force(movedline2, forcelist[i].Type, forcelist[i].Direction)
        polygon.append(movedforce2)
    
    if len(polygon) == 2:
        offset = 0.3
        
        """ line1 = polygon[0].Line
        line1start = gh.EndPoints(line1)[0]
        rotatedline1 = gh.Rotate(line1, math.pi/2, line1start)[0]
        offsetline1 = gh.Amplitude(rotatedline1,offset)
        movedline1 = gh.Move(line1, offsetline1)[0]
        polygon[0].Line = movedline1

        centroid = gh.DivideCurve(line1, 2, False)[0][1]
        
        line2 = polygon[1].Line
        line2start = gh.EndPoints(line2)[0]
        rotatedline2 = gh.Rotate(line2, math.pi/2, line2start)[0]
        offsetline2 = gh.Amplitude(rotatedline2, offset)
        movedline2 = gh.Move(line2, offsetline2)[0]
        polygon[1].Line = movedline2 """

        line1 = polygon[0].Line
        line1start = line1.start
        rotatedline1 = Rotate(line1, math.pi/2, UnitZ(line1start))
        offsetline1 = Amplitude(rotatedline1,offset)
        movedline1 = Move(line1, offsetline1)
        polygon[0].Line = movedline1

        crv = OCCNurbsCurve.from_line(line1)

        parameters, div_pts = crv.divide(2, True)
        centroid = div_pts[1]
        
        line2 = polygon[1].Line
        line2start = line2.start
        rotatedline2 = Rotate(line2, math.pi/2, UnitZ(line2start))
        offsetline2 = Amplitude(rotatedline2, offset)
        movedline2 = Move(line2, offsetline2)
        polygon[1].Line = movedline2
        
    else:
        polyline = []
        for force in polygon:
            polyline.append(force.Line.start)
        """ polygonjoined = gh.JoinCurves(polyline, True)
        centroid = gh.Area(polygonjoined)[1] """
        ########################## using polygon instead of polyline ################ 
        polygonjoined = Polygon(polyline)
        centroid = polygonjoined.centroid
    
    movedpolygoncenter = drawlocation
    """ translation = gh.Vector2Pt(centroid, movedpolygoncenter)[0] """
    translation = Vector.from_start_end(centroid, movedpolygoncenter)
    
    for force in polygon:
        """ force.Line = gh.Move(force.Line, translation)[0] """
        force.Line = Move(force.Line, translation)
    
    return polygon

def GetNodeIndex (node, currentshape):
    for i in range(0,len(currentshape.Nodes)):
        if currentshape.Nodes[i] == node:
            return i

def SearchArea (node, assembly, searchradius):
    
    # PURPOSE : Check for neighboring nodes. 
    
    # INPUT : 1) assembly.node instance  2) assembly instance  3) searchradius (float)
    # OUTPUT : list of neighboring nodes 
    
    allnodes = assembly.Nodes
    
    neighborpts = [] #list of neighboring nodes
    for point in allnodes:
        """ dist = gh.Distance(node.Coordinate, point.Coordinate)
        if (dist>0 and dist<searchradius): #exclude self, but collect other nodes within search area
            neighborpts.append(point) """
        dist = node.Coordinate.distance_to_point(point.Coordinate)
        if (dist>0 and dist<searchradius): 
            neighborpts.append(point)
    
    if len(neighborpts)>0:
        neighbor = True
    else:
        neighbor = False
    
    return (neighbor, neighborpts)

def SearchConnection (node, assembly, searchradius):
    
    # PURPOSE : Check for connecting points. 
    
    # INPUT : 1) assembly.node instance  2) assembly instance  3) searchradius (float)
    # OUTPUT : 1) Closest node  2) intersection point
    
    allnodes = assembly.Nodes
    
    allpts = []
    for node in allnodes:
        allpts.append(node.Coordinate)
    """ searcharea = gh.Circle(node.Coordinate, searchradius) """
    plane = Plane(node.Coordinate, [0, 0, 1])
    searcharea = Circle(plane, searchradius)
    
    # 1. find closest point
    """ closestpt = gh.ClosestPoint(node.Coordinate, allpts)[0] """
    closestpt = ClosestPoint(node.Coordinate, allpts)

    for pt in allnodes:
        if pt.Coordinate == closestpt:
            closestnode = pt
    
    # 2. Get LOA
    nodeLOA = GetLineOfAction(node)
    closestnodeLOA = GetLineOfAction(closestnode)
    
    # 3. Get Intersections
    intersection = [] #returns: 1) intersection point  2) curve id of node  3) curve id of closestnode
    for i in range(0, len(nodeLOA)):
        for j in range(0, len(closestnodeLOA)):
            point = UniDirVectorIntersection (node.Coordinate, nodeLOA[i], closestnode.Coordinate, closestnodeLOA[j])
            """ if point != None and gh.PointinCurves(point, [assembly.Boundary, searcharea])[0] == 2: """
            if point is not None:
                if cg.is_point_in_polygon_xy(point, assembly.Boundary) or cg.is_point_in_circle(point, searcharea) == True:
                    intersection.append([point, i, j])
    
    return intersection

def FindClosestNodes (node, assembly, count):
    
    # PURPOSE : Find the closest node.
    # INPUT : 1) ssembly.node instance  2)assembly instance  
    # OUTPUT : 1)node  2)index  3) distance
    
    # collect only active nodes
    allothernodes = []
    for allnodes in assembly.Nodes:
        if allnodes.NodeState == 1:
            allothernodes.append(allnodes)
    
    #get coordinates of collected nodes
    allothercoordinates = []
    for othernode in allothernodes:
        allothercoordinates.append(othernode.Coordinate)
    
    #find closest point
    """ closestpt = gh.ClosestPoints(node.Coordinate, allothercoordinates, count+1) """
    closestpt = ClosestPoints(node.Coordinate, allothercoordinates, count+1)
    index = closestpt[1][1:] #CP index: exclude self from closest point list 
    
    
    closestnodes = []
    
    for i in index:
        closestnodes.append(allothernodes[i])
    
    
    
    return (closestnodes) #list.

def FindClosestProjection (node, assembly, count):
    
    # PURPOSE : Find the closest node.
    # INPUT : 1) assembly.node instance  2)assembly instance  
    # OUTPUT : 1) assembly.node instance  2)coordinate of the closest projection
    
    nodeLOA = GetLineOfAction(node)[0]
    
    # list of closest nodes
    closestnodes = FindClosestNodes(node, assembly, count)
    
    # find projection of the current node and the cloesest node... SAME ORDER AS closestnodes
    projections = []
    for closestnode in closestnodes:
        closestnodeLOA = GetLineOfAction(closestnode)[0]
        point = UniDirVectorIntersection (node.Coordinate, nodeLOA, closestnode.Coordinate, closestnodeLOA)
        projections.append(point)
    
    
    if len(projections) == 1 and projections[0] == None:
        return (None, None)
    
    else:
        """ closestprojection = gh.ClosestPoint(node.Coordinate, projections) """
        closestprojection = ClosestPoint(node.Coordinate, projections)

        closestprojectionindex = closestprojection[1] # indice of closest projections (point coordinate).
        
        if closestprojectionindex == None:
            return (None, None)
        else:
            intersection = projections[closestprojectionindex] # coordinate of closest projection
            return (closestnodes[closestprojectionindex], intersection) # closest node and intersection

def FindClosestLink (node, assembly):
    
    # PURPOSE : Find the closest node.
    # INPUT : 1) assembly.node instance  2)assembly instance  
    # OUTPUT : 1) assembly.node instance  2)coordinate of the closest projection
    
    nodeLOA = GetLineOfAction(node)[0]
    
    # list of closest nodes
    closestnodes = FindClosestNodes(node, assembly, 6)
    
    # find projection of the current node and the cloesest node... SAME ORDER AS closestnodes
    projections = []
    for closestnode in closestnodes:
        closestnodeLOA = GetLineOfAction(closestnode)[0]
        point = UniDirVectorIntersection (node.Coordinate, nodeLOA, closestnode.Coordinate, closestnodeLOA)
        if point == None:
            """ point = rg.Point3d(10000,0,0) """
            point = Point(10000,0,0)
        projections.append(point)
    
    """ closestprojection = gh.ClosestPoints(node.Coordinate, projections, 6) """
    closestprojection = ClosestPoints(node.Coordinate, projections, 6)
    
    if closestprojection[1] == None:
        return (None, None, None)
    
    closestprojectionindex = closestprojection[1][0] # indice of closest projections (point coordinate).
    
    if projections[closestprojectionindex].X == 10000:
        return (None, None, None)
        
    else:
        intersection = projections[closestprojectionindex] # coordinate of closest projection
        return (closestnodes[closestprojectionindex], intersection) # closest node and intersection

def FlipOneFindLink (node, assembly):
    
    #flip direction  
    
    NodeForces = []
    for i in range(0, len(node.Forces)):
        if node.Forces[i].Type != 3: #if the force is a temporary force
            NodeForces.append(node.Forces[i])
    TempForce = GetTempForce (node)[0]
    TempForce.Direction *= -1
    NodeForces.append(TempForce)
    node.Forces = NodeForces
    node = RearrangeForces(node)
    
    nodeLOA = GetLineOfAction(node)[0]
    
    # list of closest nodes
    closestnodes = FindClosestNodes(node, assembly, 5)
    
    # find projection of the current node and the cloesest node... SAME ORDER AS closestnodes
    projections = []
    for closestnode in closestnodes:
        closestnodeLOA = GetLineOfAction(closestnode)[0]
        point = UniDirVectorIntersection (node.Coordinate, nodeLOA, closestnode.Coordinate, closestnodeLOA)
        if point == None:
            """ point = rg.Point3d(10000,0,0) """
            point = Point(10000,0,0) 
        projections.append(point)
    
    """ closestprojection = gh.ClosestPoints(node.Coordinate, projections, 6) """
    closestprojection = ClosestPoints(node.Coordinate, projections, 6)
    closestprojectionindex = closestprojection[1][0] # indice of closest projections (point coordinate).
    
    if closestprojection[1] == None or projections[closestprojectionindex].X == 10000:
        return (None, None, None)
        
    else:
        intersection = projections[closestprojectionindex] # coordinate of closest projection
        return (closestnodes[closestprojectionindex], intersection) # closest node and intersection

def FlipTwoFindLink (node, assembly):
    
    #flip direction  
    node = FlipTempForceDir(node)
    
    nodeLOA = GetLineOfAction(node)[0]
    
    # list of closest nodes
    closestnodes = FindClosestNodes(node, assembly, 5)
    
    # find projection of the current node and the cloesest node... SAME ORDER AS closestnodes
    projections = []
    for closestnode in closestnodes:
        closestnodeLOA = GetLineOfAction(closestnode)[0]
        point = BiDirVectorIntersection (node.Coordinate, nodeLOA, closestnode.Coordinate, closestnodeLOA)
        if point == None:
            """ point = rg.Point3d(10000,0,0) """
            point = Point(10000,0,0) 
        projections.append(point)
    
    """ closestprojection = gh.ClosestPoints(node.Coordinate, projections, 6) """
    closestprojection = ClosestPoints(node.Coordinate, projections, 6)
    closestprojectionindex = closestprojection[1][0] # indice of closest projections (point coordinate).
    
    if closestprojection[1] == None or projections[closestprojectionindex].X == 10000:
        return (None, None, None)
        
    else:
        intersection = projections[closestprojectionindex] # coordinate of closest projection
        return (closestnodes[closestprojectionindex], intersection) # closest node and intersection

def GetClosestRayXRay (node, ray, listofrays):
    
    intersections = []
    
    for rays in listofrays:
        """ point = gh.CurveXCurve(ray, rays)[0] """
        point = intersection_line_line(ray, rays)[0]
        if point == None:
            point = Point3d(10000,0,0)
        intersections.append(point)
    
    """ closestintersection = gh.ClosestPoints(node.Coordinate, intersections) """
    closestintersection = ClosestPoints(node.Coordinate, intersections)
    closestintersectionindex = closestintersection[1][0] # indice of closest projections (point coordinate).
    
    if closestintersection[1] == None or intersections[closestintersectionindex].X == 10000:
        return (None)
    
    else:   
        return (closestintersectionindex) # closest intersection indice

def ConnectTwoForces (node1, node2, currentshape):

    closestprojection = FindClosestLink (node1, currentshape)
    projection = closestprojection[1] # point
    
    # Reconstruct Node1
    node1tf = GetTempForce(node1)[0]
    force1 = assembly.Force(node1tf.Line, 4, node1tf.Direction)
    forcelist1 = []
    for i in range(0, len(node1.Forces)):
        if node1.Forces[i].Type != 3:
            forcelist1.append(node1.Forces[i])
    forcelist1.append(force1)
    node1.Forces = forcelist1
    node1 = RearrangeForces(node1) #new node
    node1.NodeState = 0
    
    # Reconstruct Node2
    node2tf = GetTempForce(node2)[0]
    force2 = assembly.Force(node2tf.Line, 4, node2tf.Direction)
    forcelist2 = []
    for i in range(0, len(node2.Forces)):
        if node2.Forces[i].Type != 3:
            forcelist2.append(node2.Forces[i])
    forcelist2.append(force2)
    node2.Forces = forcelist2
    node2 = RearrangeForces(node2) #new node
    node2.NodeState = 0
    
    # Make New Node
    newnodeforcelist = []
    
    movedforce1 = TransferMemberForce(force1, projection)
    movedforce2 = TransferMemberForce(force2, projection)
    
    newnodeforcelist.append(movedforce1)
    newnodeforcelist.append(movedforce2)
    newnode = assembly.Node(projection, 5, newnodeforcelist)
    newnode = EquilibrateNode(newnode)
    newnode = RearrangeForces(newnode) #new node 1
    nodelist = currentshape.Nodes
    nodelist.append(newnode)
    newnode.NodeState = 1
    
    # Make new Members
    memberlist = currentshape.Members
    """ newmember1 = gh.Line(node1.Coordinate, projection)
    newmember2 = gh.Line(node2.Coordinate, projection) """
    newmember1 = Line(node1.Coordinate, projection)
    newmember2 = Line(node2.Coordinate, projection)
    memberlist.append(newmember1)
    memberlist.append(newmember2)
    
    return currentshape

def ConnectTwoForcesDirect (node1, node2, currentshape):
    
    projection = UniDirVectorIntersection (node1.Coordinate, GetLineOfAction(node1), node2.Coordinate, GetLineOfAction(node2))
    print (projection)
    
    # Reconstruct Node1
    node1tf = GetTempForce(node1)[0]
    force1 = assembly.Force(node1tf.Line, 4, node1tf.Direction)
    forcelist1 = []
    for i in range(0, len(node1.Forces)):
        if node1.Forces[i].Type != 3:
            forcelist1.append(node1.Forces[i])
    forcelist1.append(force1)
    node1.Forces = forcelist1
    node1 = RearrangeForces(node1) #new node
    node1.NodeState = 0
    
    # Reconstruct Node2
    node2tf = GetTempForce(node2)[0]
    force2 = assembly.Force(node2tf.Line, 4, node2tf.Direction)
    forcelist2 = []
    for i in range(0, len(node2.Forces)):
        if node2.Forces[i].Type != 3:
            forcelist2.append(node2.Forces[i])
    forcelist2.append(force2)
    node2.Forces = forcelist2
    node2 = RearrangeForces(node2) #new node
    node2.NodeState = 0
    
    # Make New Node
    newnodeforcelist = []
    
    movedforce1 = TransferMemberForce(force1, projection)
    movedforce2 = TransferMemberForce(force2, projection)
    
    newnodeforcelist.append(movedforce1)
    newnodeforcelist.append(movedforce2)
    newnode = assembly.Node(projection, 5, newnodeforcelist)
    newnode = RearrangeForces(newnode)
    newnode = EquilibrateNode(newnode)
    nodelist = currentshape.Nodes
    nodelist.append(newnode)
    newnode.NodeState = 1
    
    # Make new Members
    memberlist = currentshape.Members
    """ newmember1 = gh.Line(node1.Coordinate, projection)
    newmember2 = gh.Line(node2.Coordinate, projection) """
    newmember1 = Line(node1.Coordinate, projection)
    newmember2 = Line(node2.Coordinate, projection)
    memberlist.append(newmember1)
    memberlist.append(newmember2)
    
    return currentshape



def ResolveTwoForces (currentshape):
    # assumes there are only 2 nodes left
    activenodes = GetActiveNodes(currentshape)
    
    if len(activenodes) == 2:
        node1 = activenodes[0]
        node2 = activenodes[1]
        
        tforce1 = GetTempForce(node1)[0]
        tforce1line = tforce1.Line
        
        """ tforce1vector = gh.Vector2Pt(gh.EndPoints(tforce1line)[0],gh.EndPoints(tforce1line)[1], True)
        vector1 = gh.Vector2Pt(node1.Coordinate, node2.Coordinate, True) """
        tforce1vector = Vector.from_start_end(tforce1line.start, tforce1line.end)
        tforce1vector.unitize()
        vector1 = Vector.from_start_end(node1.Coordinate, node2.Coordinate)
        vector1.unitize()
        
        # 
        if round(tforce1vector[0][0],3) == round(vector1[0][0],3):
            """ newmember = gh.Line(node1.Coordinate, node2.Coordinate) """
            newmember = Line(node1.Coordinate, node2.Coordinate)
            memberlist = currentshape.Members
            memberlist.append(newmember)
            
            node1tf = GetTempForce(node1)[0]
            force1 = assembly.Force(node1tf.Line, 4, 1)
            forcelist1 = []
            for i in range(0, len(node1.Forces)):
                if node1.Forces[i].Type != 3:
                    forcelist1.append(node1.Forces[i])
            forcelist1.append(force1)
            node1.Forces = forcelist1
            node1 = RearrangeForces(node1) #new node
            node1.NodeState = 0
        
            node2tf = GetTempForce(node2)[0]
            force2 = assembly.Force(node2tf.Line, 4, 1)
            forcelist2 = []
            for i in range(0, len(node2.Forces)):
                if node2.Forces[i].Type != 3:
                    forcelist2.append(node2.Forces[i])
            forcelist2.append(force2)
            node2.Forces = forcelist2
            node2 = RearrangeForces(node2) #new node
            node2.NodeState = 0
        
        else:
            """ newmember = gh.Line(node1.Coordinate, node2.Coordinate) """
            newmember = Line(node1.Coordinate, node2.Coordinate)
            memberlist = currentshape.Members
            memberlist.append(newmember)
            
            node1tf = GetTempForce(node1)[0]
            force1 = assembly.Force(node1tf.Line, 4, -1)
            forcelist1 = []
            for i in range(0, len(node1.Forces)):
                if node1.Forces[i].Type != 3:
                    forcelist1.append(node1.Forces[i])
            forcelist1.append(force1)
            node1.Forces = forcelist1
            node1 = RearrangeForces(node1) #new node
            node1.NodeState = 0
        
            node2tf = GetTempForce(node2)[0]
            force2 = assembly.Force(node2tf.Line, 4, -1)
            forcelist2 = []
            for i in range(0, len(node2.Forces)):
                if node2.Forces[i].Type != 3:
                    forcelist2.append(node2.Forces[i])
            forcelist2.append(force2)
            node2.Forces = forcelist2
            node2 = RearrangeForces(node2) #new node
            node2.NodeState = 0
    
        return currentshape
    
    else:
        return currentshape




################################################################################
# MEMBER OPERATIONS
################################################################################

def GetMemberNodes (member, assembly):
    
    # PURPOSE : Get end nodes of a member 
    ########### INPUT  : 1) member (line object)  2) assembly instance
    ########### OUTPUT : 1) two nodes
    
    # get end points of each member
    """ endpoints = gh.EndPoints(member)
    start = endpoints[0]
    end = endpoints[1]  """

    start = member.start
    end = member.end
    
    endnodes = []
    for node in assembly.Nodes:
        if node.Coordinate == start or node.Coordinate == end:
            endnodes.append(node)
    
    return endnodes

def GetMemberForce (member, assembly):
    
    # PURPOSE : Get the force amplitude and type for the member 
    ########### INPUT  : 1) member (line object)
    ########### OUTPUT : 1) force amplitude  2) force type
    
    membervector = LineToVector(member)[0]
    node = GetMemberNodes(member, assembly)[0] #one of the nodes
    
    for force in node.Forces:
        #print "node index,", GetNodeIndex(node, assembly)
        if force.Type == 4:
            
            vector = LineToVector(force.Line)[0]
            """ crossproduct = gh.CrossProduct(membervector, vector)[1] """
            cross_vec = cg.cross_vectors(membervector, vector)
            crossproduct = cross_vec.length

            if crossproduct < 0.001 :
                print (crossproduct)
                """ amplitude = gh.VectorLength(vector) """
                amplitude = vector.length

                direction = force.Direction
                
            #else:
                #amplitude = None
                #direction = None
    return (amplitude, direction)
    
