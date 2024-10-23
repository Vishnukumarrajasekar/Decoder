import random

class RandomUtility:
    def __init__(self, s):
        random.seed(s)
        
    def Btwn(self, lb, ub):
        return random.randint(lb, ub)
        
    def Float(self, lb, ub):
        #return random.uniform(lb, ub)
        values = [lb + (ub - lb) * i / 9 for i in range(10)]
        return random.choice(values)
    

    
MyRandom = RandomUtility(1)
