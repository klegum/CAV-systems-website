import math
import matplotlib.pyplot as plt
import pygame, sys
from statistics import stdev
import csv

reactionTime = 1
cccDelay = 0.2
ah = 0.4
bh = 0.4
alpha = 2
beta = 0.4
vmax = 30
hst = 15
hgo = 100
car_length = 5
track_length = 250
c = 0.05
amin = -6
amax = 3
stepsPerSecond = 50
speedOfAnimation = 5
StabilityThreshold = 3

absolutePositions = []

class Car:
    cars = []
    def __init__(self, position):
        # currently all parameters are constant, but these can be changed per vehicle
        self.vmax = vmax
        self.amax = amax
        self.amin = amin
        self.hst = hst
        self.hgo = hgo
        self.car_length = car_length
        self.c = c
        self.reactionTime = reactionTime
        self.cccDelay = cccDelay
        self.id = 0

        self.headwayHistoryTau = [0 for x in range(round(stepsPerSecond*self.reactionTime))]
        self.velocityHistoryTau = [0 for x in range(round(stepsPerSecond*self.reactionTime))]
        self.headwayHistorySigma = [0 for x in range(round(stepsPerSecond*self.cccDelay))]
        self.velocityHistorySigma = [0 for x in range(round(stepsPerSecond*self.cccDelay))]
        self.distance_travelled = position
        self.velocity = 0
    
    def sort_cars():
        for i in range(len(Car.cars) - 1):
            for j in range(len(Car.cars) - 1):
                if Car.cars[j].distance_travelled > Car.cars[j + 1].distance_travelled:
                    temp = Car.cars[j + 1]
                    Car.cars[j + 1] = Car.cars[j]
                    Car.cars[j] = temp

        Car.cars.reverse()
        for human in Car.cars:
            print(human.distance_travelled)
        
    def selectCarInFront(self, car_in_front):
        self.next_vehicle = car_in_front

    def selectObjectInFront(self, absPos):
        counter = 0
        for object in absPos:
            if (object.id == self.id):
                return absPos[counter-1]
            else:
                counter += 1


    def getHeadway(self, absPos):
        if self.selectObjectInFront(absPos) == absPos[-1]: # changed
            x = (
                self.selectObjectInFront(absPos).distance_travelled # changed
                + track_length
                - self.distance_travelled
            ) - self.car_length

        else:
            x = (
                self.selectObjectInFront(absPos).distance_travelled - self.distance_travelled # changed
            ) - self.car_length

        while x < 0:
            x += track_length
        return x
    
    def getPosition(self):
        return self.distance_travelled % track_length

    def updateVelocity(self, absPos):
        self.velocityHistoryTau.insert(0, self.velocity)
        self.velocityHistorySigma.insert(0, self.velocity)
        self.velocityHistoryTau.pop()
        self.velocityHistorySigma.pop()
        self.velocity += (self.getAcceleration(absPos) / stepsPerSecond)
        return self.velocity
    
    def getAcceleration(self,absPos):
        a = self.optimalAcceleration() + self.velocityDelta(absPos)
        if (a <= self.amin - self.c):
            return self.amin
        elif (a < self.amin + self.c):
            return a + ((self.amin - a + self.c) ** 2 / (4 * self.c))
        elif (a <= self.amax - self.c):
            return a
        elif (a < self.amax + self.c):
            return a - ((self.amax - a - self.c) ** 2 / (4 * self.c))
        else:
            return self.amax
        
    def getHeadwayTau(self):
        return self.headwayHistoryTau[-1]
    
    def getVelocityTau(self):
        return self.velocityHistoryTau[-1]
    
    def getHeadwaySigma(self):
        return self.headwayHistorySigma[-1]
    
    def getVelocitySigma(self):
        return self.velocityHistorySigma[-1]

class Human(Car):
    def __init__(self, position):
        self.ah = ah
        self.bh = bh
        self.type = "human"
        Car.__init__(self, position)

    def optimalVelocity(self):
        headway = self.getHeadwayTau()
        if headway <= self.hst:
            return 0
        elif headway >= self.hgo:
            return self.vmax
        elif (headway > self.hst) and (headway < self.hgo):
            return (self.vmax / 2) * (1 - (math.cos(math.pi * (headway - self.hst) / (self.hgo - self.hst))))
    
    def optimalAcceleration(self):
        return self.ah*(self.optimalVelocity() - self.getVelocityTau())

    def velocityDelta(self, absPos):
        if (self.selectObjectInFront(absPos).type == "human") or (self.selectObjectInFront(absPos).type == "autonomous"):
            return self.bh*(self.selectObjectInFront(absPos).getVelocityTau() - self.getVelocityTau()) # changed
        elif (self.selectObjectInFront(absPos).type == "traffic_light"):
            return self.bh*(0 - self.getVelocityTau())
    
    def __str__(self):
        x = self.optimalVelocity()
        return f"Human OV - {self.optimalVelocity()} and id = {self.id}"
    
class Autonomous(Car):
    def __init__(self, position):
        self.alpha = alpha
        self.beta = beta
        self.carsSeen = []
        self.type = "autonomous"
        Car.__init__(self, position)

    def cascade(self, absPos):
        index = 0
        selfIndex = 0
        for object in absPos:
            if (object.id == self.id):
                selfIndex = index
            else:
                index += 1
        counter = selfIndex - 1
        if (self.selectObjectInFront(absPos).type != "autonomous") and (self.selectObjectInFront(absPos).type != "traffic_light"): # changed
            while (absolutePositions[counter].type != "autonomous") and (absolutePositions[counter].type != "traffic_light"):
                self.carsSeen.append(counter)
                if counter == (len(absPos)-1):
                    counter = 0
                else:
                    counter -= 1
            self.carsSeen.append(counter)
        else:
            self.carsSeen.append(counter)

    def optimalVelocity(self):
        headway = self.getHeadwaySigma()
        if headway <= self.hst:
            return 0
        elif headway >= self.hgo:
            return self.vmax
        else:
            return (self.vmax / 2) * (1 - (math.cos(math.pi * (headway - self.hst) / (self.hgo - self.hst))))
    
    def optimalAcceleration(self):
        return self.alpha*(self.optimalVelocity() - self.getVelocitySigma())

    def velocityDelta(self, absPos): # different velocity delta for autonomous vehicles
        tempDelta = 0
        for human_index in self.carsSeen:
            if absPos[human_index].type == "human":
                tempDelta = tempDelta + (absPos[human_index].bh * (absPos[human_index].getVelocitySigma() - self.getVelocitySigma()))
            elif absPos[human_index].type == "autonomous":
                tempDelta = tempDelta + (absPos[human_index].beta * (absPos[human_index].getVelocitySigma() - self.getVelocitySigma()))
            elif absPos[human_index].type == "traffic_light":
                tempDelta = tempDelta + (self.beta * (0 - self.getVelocitySigma()))
        return tempDelta
    
    def __str__(self):
        x = self.optimalVelocity()
        return f"AUTOMATED OV - {self.optimalVelocity()} and id = {self.id}"
    
class TrafficLight():
    def __init__(self,position):
        self.position = position
        self.distance_travelled = position
        self.velocity = 0
        self.type = "traffic_light"
        self.state = True # false is green, red is true
        self.id = 0
        self.time = 20 # time to stay red/green for (in seconds)
        self.counter = 1
    
    def getPosition(self):
        return self.position
    
    def setState(self,timestep):
        timestepMin = (self.time*self.counter*stepsPerSecond) - (stepsPerSecond-1)
        timestepMax = (self.time*self.counter*stepsPerSecond) + (stepsPerSecond-1)
        if (timestepMin < timestep) and (timestep < timestepMax):
            if self.state == True:
                self.state = False
            else:
                self.state = True
            self.counter += 1
            print("changed")
    
    def __str__(self):
        return f"Traffic Light id = {self.id}"
    
def updatePositions():
    tempObstacles = obstacles.copy()
    for obstacle in tempObstacles:
        if (obstacle.type == "traffic_light") and (obstacle.state == False):
            tempObstacles.remove(obstacle)
    tempAbsPos = Car.cars + tempObstacles
    tempAbsPos.sort(key=lambda x: x.getPosition(), reverse=True)
    return(tempAbsPos)
        
def setId():
    for i in range(len(absolutePositions)):
        absolutePositions[i].id = i

def linkCars():
    for i in range(len(Car.cars) - 1):
        Car.cars[i + 1].selectCarInFront(Car.cars[i])
    Car.cars[0].selectCarInFront(Car.cars[-1])
    for car in Car.cars:
        print(car.next_vehicle)
    '''for i in range(len(Car.cars) - 1):
        if Car.cars[i].type == "autonomous":
            Car.cars[i].cascade()'''

def allCascade(absPos):
    for i in absPos:
        if i.type == "autonomous":
            i.cascade(absPos)

def allStates(timestep):
    for i in absolutePositions:
        if i.type == "traffic_light":
            i.setState(timestep)

def main():
    counter = 0
    headwayData = []
    tempHeadway = []
    velocityData = []
    tempVelocity = []
    tempVelocityA, velocityA = [],[]
    tempHeadwayA, headwayA = [],[]
    tempAccelA, accelA = [],[]
    accelData = []
    tempAccel = []
    tempPosition = []
    global positionData
    positionData = []
    stdevs = []
    finished = False

    fig, ax = plt.subplots(
        ncols=1, nrows=3, figsize=(10, 5.4), layout="constrained", sharex=True
    )

    while True:
        for x in range(stepsPerSecond):
            counter += 1

            absolutePositions = updatePositions()
            allStates(counter)
            allCascade(absolutePositions)

            #print(absolutePositions)

            for car in Car.cars:
                if car.type == "human":
                    tempVelocity.append(car.velocity)
                    tempAccel.append(car.getAcceleration(absolutePositions))
                    tempHeadway.append(car.getHeadway(absolutePositions))
                elif car.type == "autonomous":
                    tempVelocityA.append(car.velocity)
                    tempAccelA.append(car.getAcceleration(absolutePositions))
                    tempHeadwayA.append(car.getHeadway(absolutePositions))
                tempPosition.append(car.distance_travelled % track_length)

                car.updateVelocity(absolutePositions)

                car.distance_travelled += (car.velocity/stepsPerSecond)
                car.headwayHistoryTau.insert(0, car.getHeadway(absolutePositions))
                car.headwayHistorySigma.insert(0, car.getHeadway(absolutePositions))
                car.headwayHistoryTau.pop()
                car.headwayHistorySigma.pop()

                if car.getHeadway(absolutePositions) < car.car_length:
                    print(f"CRASH DETECTED at {(counter*stepsPerSecond) + x} timesteps")
                    finished = True # jsut for testing :)                

            velocityData.append(tempVelocity)
            accelData.append(tempAccel)
            headwayData.append(tempHeadway)
            positionData.append(tempPosition)
            velocityA.append(tempVelocityA)
            headwayA.append(tempHeadwayA)
            accelA.append(tempAccelA)

            stdevs.append(stdev(tempHeadway + tempHeadwayA))
            
            if stdev(tempHeadwayA + tempHeadway) < StabilityThreshold:
                print (f"ROBUST STABILITY ACHIEVED AT {(counter*stepsPerSecond) + x} timesteps")
                finished = False # just for testing :)

            tempHeadwayA = []
            tempAccelA = []
            tempVelocityA = []
            tempAccel = []
            tempVelocity = []
            tempPosition = []
            tempHeadway = []
        usr = input()
        if usr == "show" or finished:
            ax[0].plot(range(counter), velocityData)
            ax[0].plot(range(counter), velocityA, linestyle='dashed')
            ax[0].set_ylabel("Velocity")
            ax[1].plot(range(counter), accelData)
            ax[1].plot(range(counter), accelA, linestyle='dashed')
            ax[1].set_ylabel("Acceleration")
            ax[2].plot(range(counter), headwayData)
            ax[2].plot(range(counter), headwayA, linestyle='dashed')
            ax[2].plot(range(counter), stdevs, linewidth=3)
            ax[2].set_ylabel("Headway")
            plt.xlabel("Timesteps")
            plt.show()
            animate()
        elif usr == "end":
            animate()
        elif usr == "data":

            allVelocities = [vsHuman+vsAuto for vsHuman, vsAuto in zip(velocityData, velocityA)]
            velocityHeaders = [f"Velocity{x+1}" for x in range(len(velocityData[0]))] + [f"VelocityAV{x+1}" for x in range(len(velocityA[0]))]
            allAccels = [asHuman+asAuto for asHuman, asAuto in zip(accelData, accelA)]
            accelHeaders = [f"Accel{x+1}" for x in range(len(velocityData[0]))] + [f"AccelAV{x+1}" for x in range(len(velocityA[0]))]
            allHeadways = [hsHuman+hsAuto for hsHuman, hsAuto in zip(headwayData, headwayA)]
            headwayHeaders = [f"Headway{x+1}" for x in range(len(headwayData[0]))] + [f"HeadwayAV{x+1}" for x in range(len(velocityA[0]))]

            with open('CAV_data.csv', 'w', newline='') as cavData:
                dataWriter = csv.writer(cavData)

                headers = ['Time'] + velocityHeaders + accelHeaders + headwayHeaders + ["StdDV(Headway)"]
                dataWriter.writerow(headers)
            
                for x in range(len(velocityData)):
                    row = [x/stepsPerSecond] + allVelocities[x] + allAccels[x] + allHeadways[x] + [stdevs[x]]
                    dataWriter.writerow(row)

        print([str(x) for x in absolutePositions])

Car.cars = [Autonomous(0), Human(100), Human(50)]
obstacles = []

Car.sort_cars()
linkCars()
absolutePositions = updatePositions()
setId()
allCascade(absolutePositions)
print(absolutePositions)

def animate(speed=speedOfAnimation):
    pygame.init()

    class humanSprite:
        def __init__(self, position, colour):
            self.x, self.y = position
            self.colour = colour

        def display(self):
            pygame.draw.circle(screen, self.colour, (self.x, self.y), 30, width=0)

    clock = pygame.time.Clock()

    screen_width = 720
    screen_height = 720
    screen = pygame.display.set_mode((screen_width, screen_height))

    humanSprites = [humanSprite((0, 0), "RED") if car.type == "human" else humanSprite((0,0), "BLUE") for car in Car.cars]

    bg = pygame.image.load("RingRoad.png")
    bg = pygame.transform.scale(bg, (screen_width, screen_height))
    r = 300
    for timestepsPassed in range(len(positionData) - 1):
        if timestepsPassed%speed == 0:
            screen.blit(bg, (0, 0)) 
        else:
            continue

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        for pos, humanCar in enumerate(humanSprites):
            theta = (360/track_length)*positionData[timestepsPassed][pos]
            humanCar.x, humanCar.y = (
                r * math.cos(math.radians(theta)) + 360,
                720 - ((r * math.sin(math.radians(theta)) + 360)),
            )
            humanCar.display()
            

        pygame.display.flip()
        clock.tick(60)
    exit()

main()