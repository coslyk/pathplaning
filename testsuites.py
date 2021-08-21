

from shapely.geometry import Point, Polygon, LineString # import geometric elements
from descartes.patch import PolygonPatch # import ability to draw geometric elements

class TestEnvironment():
    
    def __init__(self, name, start, goal, limits, scene):
        """
            Constructor
        
            :name: name of test
            :start: start position as list
            :goal: goal position as list
            :limits: list of list containing lower and upper limit
            :scene: scene description (dict containing geometric primitives from shapely)
        
            e.g.:
            ...
            testEnv = TestEnvironment("Test", [1,1], [10,10], [[0,0],[20,20]], scene)
            ...
        
        """
        
        self.name = name
        self.start = start
        self.goal = goal
        self.limits = limits
        self.scene = scene
        

        
testSuite = list()

commonStart = [2,20]
commonGoal  = [19,2]

simpleField = dict() # using a dict as structure to store the scene
simpleField["obs1"] = Polygon([(6, 14), (13, 14), (13, 11), (6,11)])
simpleField["obs2"] = Polygon([(16, 5), (19, 5), (19, 3),(16,3)])
testSuite.append(TestEnvironment("simpleField", commonStart, commonGoal, [[0,25],[0,25]],simpleField))

trapField = dict()
trapField["obs1"] =   LineString([(6, 18), (6, 8), (16, 8), (16,18)]).buffer(1)
testSuite.append(TestEnvironment("trapField", commonStart, commonGoal, [[0,25],[0,25]],trapField))

special = dict()
special["base"] = LineString([(11,0),(11,18)]).buffer(0.5)
for i in range(15):
    target1 = (11-0.5*i,18-i)
    target2 = (11+0.5*i,18-i)
    special["partA"+str(i)] = LineString([(11,18-i), target1 ]).buffer(0.2)
    special["partB"+str(i)] = LineString([(11,18-i), target2 ]).buffer(0.2)
    if i % 2 == 0:
        special["candA"+str(i)] = LineString([target1, (target1[0], target1[1]+1.5)]).buffer(0.05)
        special["candB"+str(i)] = LineString([target2, (target2[0], target2[1]+1.5)]).buffer(0.05)

testSuite.append(TestEnvironment("special", commonStart, commonGoal, [[0,25],[0,25]],special))

trapField2 = dict()
trapField2["obs1"] = LineString([(6, 18), (6, 8), (16, 8), (16,18)]).buffer(0.2)
trapField2["obs2"] = Polygon([(16, 5), (19, 5), (19, 3),(16,3)])
testSuite.append(TestEnvironment("trapField2", commonStart, commonGoal, [[0,25],[0,25]],trapField2))


