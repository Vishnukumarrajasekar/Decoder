import Grammar as gr
import Assembly as assembly
import RandomUtility as ru

# This is a script that tests the grammar functionality.

#test = assembly.Assembly([], [10,10])
#
g = gr.Grammar()
b = gr.RandomComputation(g)



#
for i in range(1):
    a = b.GenerateRandomAssembly()
    print len(a.Circles)
    print a.Number
    print a.Number
    print a.Boolean
    for t in a.History.Rules:
        print t[0].Name
        print t[1]

#print ru.MyRandom.Btwn(0, 2)