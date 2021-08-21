

from shapely.geometry import Point, Polygon, LineString # import geometric elements
from descartes.patch import PolygonPatch # import ability to draw geometric elements

class CollisionChecker():

    def __init__(self, scene, limits = [[0.0,22.0],[0.0,22.0]]):
        """
        Contructor
        
        :scene: scene description (dict containing geometric primitives from shapely)
        :limits: limits of the environment
        
        """
        
        self._scene = scene
        self._limits = limits
        
    def getDim(self):
        """ 
        Return dimension of Environment (Shapely should currently always be 2)
        """
        
        return 2
    
    def getEnvironmentLimits(self):
        """ 
        Return limits of Environment
        """
        
        return list(self._limits)

    def pointInCollision(self, pos):
        """
        Return whether a configuration 
        * is inCollision -> True
        * Free -> False 
        """

        for key, value in self._scene.items():
            if value.intersects(Point(pos[0],pos[1])):              
                return True
        return False

    def lineInCollision(self, startPos, endPos):
        """ 
        Check whether a line from startPos to endPos is colliding
        """
       
        for key, value in self._scene.items():
            if value.intersects(LineString([(startPos[0],startPos[1]),(endPos[0],endPos[1])])):
                return True
        return False
    
    def drawObstacles(self, ax):
        """
        Draw obsacles in diagramm given by ax
        """
        
        for key, value in self._scene.items():
            patch = PolygonPatch(value, facecolor="red", alpha=0.8, zorder=2, label=key)
            ax.add_patch(patch)
