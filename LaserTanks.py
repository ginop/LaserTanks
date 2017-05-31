import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
import matplotlib.transforms as transforms
import importlib



class laserTank:
    def __init__(self, controlFunction):
        self.control = importlib.import_module(controlFunction)
    
    def draw(self, fig):
        ts = fig.transData
        coords = ts.transform(self.position)
        t1 = transforms.Affine2D().rotate_deg_around(coords[0], coords[1], self.orientation[0])
        t2 = transforms.Affine2D().rotate_deg_around(coords[0], coords[1], self.orientation[1])
        tA = ts + t1
        tB = tA + t2
        
        x = self.position[0]
        y = self.position[1]
        fig.add_patch(patches.Rectangle((-1.6+x, 0.8+y), 3.2, 0.4, color=self.color, transform=tA))
        fig.add_patch(patches.Rectangle((-1.6+x, -1.2+y), 3.2, 0.4, color=self.color, transform=tA))
        fig.add_patch(patches.Rectangle((-1.5+x, -1+y), 3, 2, color='black', transform=tA))
        fig.add_patch(patches.Rectangle((0+x, -0.1+y), 1, 0.2, color=self.color, transform=tB))
        fig.add_patch(patches.Circle((0+x, 0+y), radius=0.6, color=self.color, transform=tB))
    
    position = [0, 0]
    orientation = [0, 0]
    velocity = [0, 0]
    color='blue'
    hull = 100
    battery = 100
      
    
    
tank1 = laserTank('testControls')
print(tank1.control.main(1))

plt.figure()
plt.axis('equal')
plt.axis([-10, 10, -10, 10])

tank1.position = [-3, -2]
tank1.orientation = [10, -5]
tank1.draw(plt.gca())

tank1.position = [5, 2]
tank1.orientation = [160, 20]
tank1.color = 'red'
tank1.draw(plt.gca())

plt.show()