import math
import matplotlib.pyplot as plt
import pygame, sys
from statistics import stdev
from random import randint
import csv
import os

reactionTime = 1
cccDelay = 0.2
ah = 0.2
bh = 0.2
alpha = 1
beta = 0.2
vmax = 30
hst = 5
hgo = 100
car_length = 5
track_length = 3250
c = 0.05
amin = -6
amax = 3
stepsPerSecond = 50
speedOfAnimation = 6
StabilityThreshold = 3

absolutePositions = []

class Car:
    cars = []

    def __init__(self, position, specificvmax):
        # currently all parameters are constant, but these can be changed per vehicle
        self.vmax = vmax
        self.vmaxLimited = vmax # vmax with speed limits
        self.amax = amax
        self.amin = amin
        self.hst = hst
        self.hgo = hgo
        self.car_length = car_length
        self.c = c
        self.reactionTime = reactionTime
        self.cccDelay = cccDelay
        self.id = 0
        self.stoppable = True # kai's idea

        self.headwayHistoryTau = [
            0 for x in range(round(stepsPerSecond * self.reactionTime))
        ]
        self.velocityHistoryTau = [
            0 for x in range(round(stepsPerSecond * self.reactionTime))
        ]
        self.headwayHistorySigma = [
            0 for x in range(round(stepsPerSecond * self.cccDelay))
        ]
        self.velocityHistorySigma = [
            0 for x in range(round(stepsPerSecond * self.cccDelay))
        ]
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
        
    def selectCarInFront(self, car_in_front):
        self.next_vehicle = car_in_front

    def selectObjectInFront(self, absPos):
        counter = 0
        nextFound = False
        for object in absPos:
            if (object.id == self.id):
                while (not nextFound):
                    if (absPos[counter-1].type == "traffic_light"):
                        if (absPos[counter-1].getColour() == "o") and (not self.stoppable):
                            nextFound = False
                            counter -= 1
                    if absPos[counter-1].type == "traffic_light":
                        if absPos[counter-1].getColour() == "g":
                            nextFound = False
                            counter -= 1
                    nextFound = True
                    return absPos[counter-1]
            else:
                counter += 1

    def getHeadway(self, absPos):
        x = (
                self.selectObjectInFront(absPos).getPosition() - self.getPosition() # changed
            ) - self.car_length
        
        while x < 0:
            x += track_length
        return x
    
    def getVehicleHeadway(self, absPos):
        if self.next_vehicle == absPos[-1]: # changed
            x = (
                self.next_vehicle.distance_travelled # changed
                + track_length
                - self.distance_travelled
            ) - self.car_length

        else:
            x = (
                self.next_vehicle.distance_travelled - self.distance_travelled # changed
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
        if (self.velocity < 0):
            self.velocity = 0
        return self.velocity
    
    def getAcceleration(self,absPos):
        a = self.optimalAcceleration() + self.velocityDelta(absPos)
        if (a <= self.amin - self.c):
            return self.amin
        elif a < self.amin + self.c:
            return a + ((self.amin - a + self.c) ** 2 / (4 * self.c))
        elif a <= self.amax - self.c:
            return a
        elif a < self.amax + self.c:
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
    def __init__(self, position, humanspecificvmax=vmax):
        self.ah = ah
        self.bh = bh
        self.type = "human"
        Car.__init__(self, position, humanspecificvmax)

    def stoppingDist(self):
        stop = (-1 * (self.getVelocityTau())**2)/(2*self.amin) + (self.getVelocityTau() * reactionTime)
        return stop

    def optimalVelocity(self):
        headway = self.getHeadwayTau()
        if headway <= self.hst:
            return 0
        elif headway >= self.hgo:
            return self.vmaxLimited
        elif (headway > self.hst) and (headway < self.hgo):
            return (self.vmaxLimited / 2) * (1 - (math.cos(math.pi * (headway - self.hst) / (self.hgo - self.hst))))
    
    def optimalAcceleration(self):
        return self.ah * (self.optimalVelocity() - self.getVelocityTau())

    def velocityDelta(self, absPos):
        '''if self.selectObjectInFront(absPos).type == "traffic_light":
            if self.selectObjectInFront(absPos).getColour != "g":
                return 0 #Ignore velocity delta if traffic light'''
        
        if self.getHeadway(absPos) > self.hgo:
            return 0
    
        if (self.selectObjectInFront(absPos).type == "human") or (self.selectObjectInFront(absPos).type == "autonomous"):
            return self.bh*(self.selectObjectInFront(absPos).getVelocityTau() - self.getVelocityTau()) # changed
        elif (self.selectObjectInFront(absPos).type == "traffic_light"):
            return self.bh*(0 - self.getVelocityTau())
    
    def __str__(self):
        x = self.optimalVelocity()
        return f"Human OV - {self.optimalVelocity()} and id = {self.id}"
    
class Autonomous(Car):
    def __init__(self, position, autonomousspecificvmax=vmax):
        self.alpha = alpha
        self.beta = beta
        self.carsSeen = []
        self.type = "autonomous"
        Car.__init__(self, position, autonomousspecificvmax)

    def cascade(self, absPos):
        index = 0
        selfIndex = 0
        self.carsSeen = []
        for object in absPos:
            if (object.id == self.id):
                selfIndex = index
            else:
                index += 1

        counter = selfIndex - 1

        # possibly ignore all of the commented code, as this is absolute garbage :)

        '''if (self.selectObjectInFront(absPos).type != "autonomous") and (self.selectObjectInFront(absPos).type != "traffic_light"): # changed
            while (absPos[counter].type != "autonomous") and (absPos[counter].type != "traffic_light"):
                self.carsSeen.append(counter)
                if counter == (len(absPos)-1):
                    counter = 0
                else:
                    counter -= 1
            self.carsSeen.append(counter)
        else:
            self.carsSeen.append(counter)'''
        
        '''while (absPos[counter].selectObjectInFront(absPos).type != "autonomous") and (absPos[counter].selectObjectInFront(absPos).type != "traffic_light"):
            self.carsSeen.append(counter)
            counter -= 1'''

        while (absPos[counter].type != "autonomous") and (absPos[counter].type != "traffic_light"):
            self.carsSeen.append(counter)
            counter -= 1

    def stoppingDist(self):
        stop = (-1 * (self.getVelocitySigma())**2)/(2*self.amin) + (self.getVelocitySigma() * cccDelay)
        return stop

    def optimalVelocity(self):
        headway = self.getHeadwaySigma()
        if headway <= self.hst:
            return 0
        elif headway >= self.hgo:
            return self.vmaxLimited
        else:
            return (self.vmaxLimited / 2) * (1 - (math.cos(math.pi * (headway - self.hst) / (self.hgo - self.hst))))
    
    def optimalAcceleration(self):
        return self.alpha * (self.optimalVelocity() - self.getVelocitySigma())

    def velocityDelta(self, absPos): # different velocity delta for autonomous vehicles
        '''if self.selectObjectInFront(absPos).type == "traffic_light":
            if self.selectObjectInFront(absPos).getColour != "g":
                return 0 #Ignore velocity delta if traffic light'''
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
    def __init__(self,position, time=20, orangeTime=5):
        self.position = position
        self.distance_travelled = position
        self.velocity = 0
        self.type = "traffic_light"
        self.state = True # false is green, red is true
        self.orangeTime = orangeTime
        self.isOrange = True
        self.orangeSteps = self.orangeTime*stepsPerSecond
        self.lastRed = 0 # the most recent timestep the light turned red
        self.id = 0
        self.time = time # time to stay red/green for (in seconds)
        self.counter = 1

    def getColour(self):
        if self.isOrange and self.state == True:
            return "o"
        elif self.state:
            return "r"
        else:
            return "g"
    
    def getPosition(self):
        return self.position
    
    def setState(self,timestep):
        timestepMin = (self.time*self.counter*stepsPerSecond) - (stepsPerSecond-1)
        timestepMax = (self.time*self.counter*stepsPerSecond) + (stepsPerSecond-1)
        if (timestepMin < timestep) and (timestep < timestepMax):
            if self.state == True:
                self.state = False # red to green
                self.isOrange = False
                #print("red to green: " + str(timestep))
                self.counter += 1
            else:
                self.state = True # green to orange
                self.isOrange = True
                self.lastRed = timestep
                #print("green to orange: " + str(timestep))
                self.counter += 1
        timeRed = timestep - self.lastRed
        if (((self.orangeSteps)-(stepsPerSecond-1)) <= timeRed) and (timeRed <= ((self.orangeSteps)-(stepsPerSecond-1))) and (self.state == True):
            self.state = True # orange to red
            self.isOrange = False
            #print("orange to red: " + str(timestep))
        if self.getColour() == "o":
            for vehicle in Car.cars:
                posDiff = (self.getPosition() - vehicle.getPosition())
                if posDiff < 0:
                    posDiff += track_length

                if (vehicle.stoppingDist()) < (posDiff): # if stoppable
                    vehicle.stoppable = True
                else:
                    vehicle.stoppable = False
        else:
            for vehicle in Car.cars:
                vehicle.stoppable = True
    
    def __str__(self):
        return f"Traffic Light id = {self.id}"
    
class SpeedLimit():
    def __init__(self, startPos, endPos, maxSpeed):
        self.minPos = startPos
        self.maxPos = endPos
        self.speed = maxSpeed
        self.type = "speed_limit"
        # right now SpeedLimit does not need an id, and is therefore not set

    def getRange(self):
        return (self.minPos, self.maxPos, self.speed)

def currentMaxSpeed(vehicle):
    withinRange = False
    position = vehicle.getPosition()
    for obstacle in obstacles:
        if obstacle.type == "speed_limit":
            posRange = obstacle.getRange()
            if (position >= (posRange[0] - hgo)) and (position <= posRange[1]):
                withinRange = True
                new_vmax = obstacle.speed
    if (not withinRange):
        vehicle.vmaxLimited = vehicle.vmax
    else:
        vehicle.vmaxLimited = new_vmax
def allSpeedLimits(absPos):
    for vehicle in absPos:
        if (vehicle.type == "human") or (vehicle.type == "autonomous"):
            currentMaxSpeed(vehicle)

def updatePositions():
    tempObstacles = obstacles.copy()
    for element in tempObstacles:
        if element.type != "traffic_light":
            tempObstacles.remove(element)

    for obstacle in tempObstacles:
        if (obstacle.type == "traffic_light"):
            if (obstacle.getColour() == 'g'):
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
        #print(car.next_vehicle)
        '''for i in range(len(Car.cars) - 1):
            if Car.cars[i].type == "autonomous":
                Car.cars[i].cascade()'''

def allCascade(absPos):
    for i in absPos:
        if i.type == "autonomous":
            i.cascade(absPos)

def allStates(timestep, absPos):
    '''for i in absPos:
        if i.type == "traffic_light":
            i.setState(timestep)'''
    for i in obstacles:
        if i.type == "traffic_light":
            i.setState(timestep)

def main(inputah=ah, inputbh=bh):

    nAutonomous = 5
    n = 30
    cascadeLen = round(n / nAutonomous)
    bh = inputbh
    ah = inputah

    r = lambda: randint(math.ceil(-(track_length/n/2.5)), math.floor(track_length/n/2.5))
    p = lambda a: (track_length/n) * a + r()


    for x in range(1, n+1):
        if x % cascadeLen == 0:
            Car.cars.append(Autonomous(p(x)))
        else:
            Car.cars.append(Human(p(x)))

    Car.sort_cars()
    linkCars()
    absolutePositions = updatePositions()
    setId()
    allCascade(absolutePositions)

    counter = 0
    headwayData = []
    tempHeadway = []
    velocityData = []
    tempVelocity = []
    tempVelocityA, velocityA = [], []
    tempHeadwayA, headwayA = [], []
    tempAccelA, accelA = [], []
    accelData = []
    tempAccel = []
    tempPosition = []
    global trafficData
    tempTrafficData, trafficData = [], []
    global positionData
    positionData = []
    stdevs = []
    finished = False


    for times in range(50):
        for x in range(stepsPerSecond):
            counter += 1

            absolutePositions = updatePositions()
            allStates(counter, absolutePositions)
            allCascade(absolutePositions)
            allSpeedLimits(absolutePositions)

            #print(absolutePositions[2].carsSeen)

            #print(absolutePositions)
            for obstacle in obstacles:
                if obstacle.type=="traffic_light":
                    tempTrafficData.append(obstacle.getColour())

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

                '''if car.getHeadway(absolutePositions) < car.car_length:
                    if (car.selectObjectInFront(absolutePositions) == "traffic_light"):
                        if (car.selectObjectInFront(absolutePositions).getColour() == 'r'):
                            print(f"CRASH DETECTED at {(counter*stepsPerSecond) + x} timesteps")
                            finished = True # jsut for testing :)                
                    else:
                        print(f"CRASH DETECTED at {(counter*stepsPerSecond) + x} timesteps")
                        finished = True # jsut for testing :)'''

            velocityData.append(tempVelocity)
            accelData.append(tempAccel)
            headwayData.append(tempHeadway)
            positionData.append(tempPosition)
            velocityA.append(tempVelocityA)
            headwayA.append(tempHeadwayA)
            accelA.append(tempAccelA)
            trafficData.append(tempTrafficData)

            stdevs.append(stdev(tempHeadway + tempHeadwayA))

            if stdev(tempHeadwayA + tempHeadway) < StabilityThreshold:
                #print (f"ROBUST STABILITY ACHIEVED AT {(counter*stepsPerSecond) + x} timesteps")
                finished = False # just for testing :)

            tempHeadwayA = []
            tempAccelA = []
            tempVelocityA = []
            tempAccel = []
            tempVelocity = []
            tempPosition = []
            tempHeadway = []
            tempTrafficData = []

    animate()
        

#INITIALISATION




obstacles = []


trafficLightPos = []
for obstacle in obstacles:
    if obstacle.type=='traffic_light':
        trafficLightPos.append(obstacle.getPosition())


#print(absolutePositions)

def animate(speed=speedOfAnimation):
    pygame.init()

    

    class Sprite:
        def __init__(self, position, colour, type='car'):
            self.x, self.y = position
            self.colour = colour
            self.type = type

        def display(self):
            pygame.draw.circle(screen, self.colour, (self.x, self.y), 10, width=0)

    text_font = pygame.font.SysFont("Roboto", 30)

    def draw_text(text, font, text_col, coords):
        img = font.render(text, True, text_col)
        screen.blit(img, coords)

    clock = pygame.time.Clock()

    screen_width = 720
    screen_height = 720
    screen = pygame.display.set_mode((screen_width, screen_height))

   

    vehicleSprites = [
        Sprite((0, 0), "RED")
        if car.type == "human"
        else Sprite((0, 0), "BLUE")
        for car in Car.cars
    ]
    obstacleSprites = [Sprite((0,0), "ORANGE", type='traffic_light') if obstacle.type == "traffic_light" else Sprite((0,0), "YELLOW") for obstacle in obstacles]

    bg = pygame.image.load("RingRoad.png")
    bg = pygame.transform.scale(bg, (screen_width, screen_height))
    r = 300

    
    def pause():
        paused = True
        while paused:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        paused = False
                    
    frame_count = 0

    for timestepsPassed in range(len(positionData) - 1):
        if timestepsPassed % speed == 0:
            screen.blit(bg, (0, 0))
            draw_text(f"Timestep: {timestepsPassed}", text_font, (0, 0, 0), (5, 5))
            draw_text(f"Seconds: {timestepsPassed/stepsPerSecond}", text_font, (0, 0, 0), (5, 30))
            draw_text(f"Speed: {speed}", text_font, (0, 0, 0), (5, 55))
        else:
            continue

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    pause()
                if event.key == pygame.K_MINUS:
                    speed -= 1
                if event.key == pygame.K_EQUALS:
                    speed += 1

        for pos, humanCar in enumerate(vehicleSprites):
            theta = (360 / track_length) * positionData[timestepsPassed][pos]

            humanCar.x, humanCar.y = (
                r * math.cos(math.radians(theta)) + 360,
                720 - ((r * math.sin(math.radians(theta)) + 360)),
            )
            humanCar.display()
                
        for pos, obstacle in enumerate(obstacleSprites):

            if obstacle.type=='traffic_light':
                theta = (360 / track_length) * trafficLightPos[pos]
                if trafficData[timestepsPassed][pos] == 'r':
                    obstacle.colour='PINK'
                elif trafficData[timestepsPassed][pos] == 'g':
                    obstacle.colour='GREEN'
                else:
                    obstacle.colour='ORANGE'

                obstacle.x, obstacle.y = (
                r * math.cos(math.radians(theta)) + 360,
                720 - ((r * math.sin(math.radians(theta)) + 360)),
            )
                
            obstacle.display()
        clock.tick(60)
        pygame.display.flip()
        frame_count += 1
        filename = "AnimationImages/screen_%04d.png" % ( frame_count )
        pygame.image.save(screen, filename)

    os.system(r"ffmpeg -y -r 60 -f image2 -s 720x720 -i AnimationImages/screen_%04d.png -vcodec libx264 -crf 25  AnimationImages/window_video.mp4")

