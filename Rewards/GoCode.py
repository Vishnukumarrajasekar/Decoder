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

def Generate (StartShape):
    
    g = gr.Grammar()
    b = gr.RandomComputation(g)
    
    NUM = 3
    bestNum = 5
    bestScores = []
    bestDesigns = []
    
    for i in range(NUM):
        #history = []
        #Grasshopper.Instances.ActiveCanvas.Document.ExpireSolution()
        
        
        a = b.GenerateRandomAssembly(StartShape)
        
        bestDesigns.append(a)
        """
        #Grasshopper.Kernel.GH_Component.Name == "MakeAssembly":
        #for obj in Grasshopper.Kernel.GH_Component.OnPingDocument(:
        if Grasshopper.Kernel.GH_Component.Name == "MakeAssembly":
            print Grasshopper.Kernel.GH_Component.Name
            Grasshopper.Kernel.GH_Component.ExpireSolution(False)

        #Grasshopper.Instances.ActiveCanvas.Document.NewSolution(False)
        """
        
        
        
        
        
        
        """
        for t in a.History.Rules:
            c = t[2].Name, "applied at Node", t[0]
            history.append(c)
        
        if len(bestDesigns) < bestNum:
            bestDesigns.append(a)
            bestScores.append(a.Score)
        else:
            bestDesigns.append(a)
            bestScores.append(a.Score)
            
            
            
            sortedscores = gh.SortList(bestScores, bestScores)[1]
            index = gh.ItemIndex(bestScores, sortedscores)
            bestDesigns = [bestDesigns[j] for j in index]
            
            bestDesigns.pop()
            
            bestScroes = sortedscores
            bestScores.pop()

    """
    return bestDesigns #list of best performing assembly instances