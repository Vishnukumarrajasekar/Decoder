import Rhino as rc

gh

def DisplayVector (point, vector, color):

    if "display" not in globals():
        display = rc.Display.CustomDisplay(True)
    
    display.Clear()
    
    #color = rc.Display.ColorHSL(rnd.random(),1.0,0.5)
    display.AddVector(point, vector[0], color, 1)
    
    #display.ghcomp.VectorDisplayEx(Tail, Vector[0], sd.Color.Aqua, 1)