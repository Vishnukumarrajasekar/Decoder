import Rhino
import clr
clr.AddReferenceByName('Grasshopper')
import Grasshopper

gh = Rhino.RhinoApp.GetPlugInObject('Grasshopper')
if not gh.IsEditorLoaded():
    print 'Grasshopper editor is not loaded'
else:
    for doc in Grasshopper.Instances.DocumentServer.Document:
        print('document name: ' + doc.DisplayName)
        i = 0
        for obj in doc.Objects:
            print('document object (' + str(i) + '): ' + obj.Name)
            i = i + 1