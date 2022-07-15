from __future__ import annotations
from typing import List
from matplotlib import pyplot as plt

import numpy as np

class Position():
    def __init__(self,x,y,z) -> None:
        self.x = x
        self.y = y
        self.z = z
    
    def add(self,p: Position) -> Position:
        return Position(self.x + p.x, self.y + p.y, self.z + p.z)

    def multiply(self,scalar: float) -> Position:
        return Position(self.x * scalar, self.y * scalar, self.z * scalar)
    
    def magnitude(self) -> float:
        return (self.x**2 + self.y**2 + self.z**2)**0.5

    def __sub__(self,p: Position) -> Position:
        return Position(self.x - p.x, self.y - p.y, self.z - p.z)

    def __add__(self,p: Position) -> Position:
        return self.add(p)

    def __str__(self) -> str:
        return f"({self.x},{self.y},{self.z})"

    def getDeltaPosition(self,otherPositions:List[Position]) -> Position:
        deltaPosition= Position(0,0,0)

        for pj in otherPositions:
            if pj.x == None or pj.y == None or pj.z == None:
                continue
            
            d = self - pj
            mag= d.magnitude()
            
            scalar = (mag-0.6)/mag
            deltaPosition = deltaPosition + d.multiply(scalar)
        
        return Position(0,0,0)-deltaPosition

def initOtherPositions(n: int) -> List[Position]:
    positions = []
    for i in range(n):
        positions.append(Position(None,None,None))
    
    return positions

    
if __name__=="__main__":

    otherPositions=initOtherPositions(10)
    otherPositions[0] = Position(+0.4,0,0)
    otherPositions[1] = Position(-0.4,0,0)
    otherPositions[2] = Position(0,0.4,0)

    
    nx,ny= (20,20)
    x = np.linspace(-1, 1, nx)
    y = np.linspace(-1, 1, ny)

    xv, yv = np.meshgrid(x, y)

    plt.figure()
    for otherPosition in otherPositions:
        if otherPosition.x == None or otherPosition.y == None or otherPosition.z == None:
            continue

        plt.plot(otherPosition.x,otherPosition.y,'o')
    
    plt.xlabel('x')
    plt.ylabel('y')
    xs,ys,us,vs=[],[],[],[]
    for i in range(nx):
        for j in range(ny):
            p = Position(xv[i][j],yv[i][j],0)
            deltaPosition = p.getDeltaPosition(otherPositions)
            # print(f"{p} {deltaPosition}")

            # plt.quiver(xv[i][j],yv[i][j],deltaPosition.x,deltaPosition.y,scale=40,headwidth=3,headlength=1,headaxislength=1)
            xs.append(xv[i][j])
            ys.append(yv[i][j])
            us.append(deltaPosition.x)
            vs.append(deltaPosition.y)
    
    plt.quiver(xs,ys,us,vs)
        
    plt.grid()
    plt.show()