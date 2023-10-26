import math
import matplotlib.pyplot as plt

ah = 0.2
bh = 0.4
vmax = 30
hst = 5
hgo = 55
car_length = 5
track_length = 180
c = 0.05
amin = -6
amax = 3
stepsPerSecond = 10

class Human():
  def __init__(self, position):
    self.distance_travelled = position
    self.velocity = 0

  def selectCarInFront(self, car_in_front):
    self.next_vehicle = car_in_front
  
  def getHeadway(self):
    x = (self.next_vehicle.distance_travelled - self.distance_travelled) - car_length
    while x < 0:
      x += track_length
    return x
  
  def getPosition(self):
    return self.distance_travelled % 360
  
  def optimalVelocity(self, ah, bh, vmax, hst, hgo):
    headway = self.getHeadway()
    if headway <= hst:
      return 0
    elif headway >= hgo:
      return vmax
    else:
      return (vmax/2) * (1 - (math.cos(math.pi * (headway - hst) / (hgo - hst))))
  
  def updateVelocity(self):
    self.velocity += self.getAcceleration()
    return self.velocity

  def optimalAcceleration(self, vstart):
    return (self.optimalVelocity(ah, bh, vmax, hst, hgo) - vstart) / stepsPerSecond

  def getAcceleration(self):
    a = self.optimalAcceleration(self.velocity)
    if a <= amin - c:
      return amin
    elif a < amin + c:
      return a + ((amin-a+c)**2/(4*c))
    elif a <= amax - c:
      return a
    elif a < amax + c:
      return a - ((amax-a-c)**2/(4*c))
    else:
      return amax


  def __str__(self):
    x = self.optimalVelocity(ah, bh, vmax, hst, hgo)
    return f"i_x is {self.distance_travelled % 360}, delta_s is {self.distance_travelled}, OV {x}"

def linkCars(humans):
  for i in range(len(humans)-1):
    humans[i].selectCarInFront(humans[(i+1)])
  humans[-1].selectCarInFront(humans[0])
  return humans

def main(humans):
  counter = 0
  velocityData = []
  tempVelocity = []
  accelData = []
  tempAccel = []
  fig, ax = plt.subplots(ncols=1, nrows=2, figsize=(10, 5.4), layout='constrained', sharex=True)
  while True:
    for x in range(stepsPerSecond):
      counter += 1
      for car in humans:
        tempVelocity.append(car.velocity)
        tempAccel.append(car.getAcceleration())
        car.updateVelocity()
        print(f"pos: {round(car.distance_travelled%360)}")
        print(f"v: {round(car.velocity)}")
        print(f"ov: {round(car.optimalVelocity(ah, bh, vmax, hst, hgo))}")
        print(f"a: {round(car.getAcceleration())}")
        car.distance_travelled += car.velocity
      velocityData.append(tempVelocity)
      accelData.append(tempAccel)
      tempAccel = []
      tempVelocity = []
    usr = input()
    if usr == 'show':
      ax[0].plot([x for x in range(counter)], velocityData)
      ax[0].set_ylabel('Velocity')
      ax[1].plot([x for x in range(counter)], accelData)
      ax[1].set_ylabel('Acceleration')
      plt.xlabel('Timesteps')
      plt.show()
      exit()
    elif usr == 'end':
      exit()

    print([str(x) for x in humans])

humans = [Human(0), Human(30), Human(50), Human(80), Human(120), Human(160), Human(170)]

#print(humans[0].getPosition())

main(linkCars(humans))