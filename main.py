import pygame
import math
from pygame.locals import *

class Environment:

    def __init__(self, dimenions):
        self.black = (0,0,0)
        self.white = (255,255,255)
        self.green = (0,255,0)
        self.blue = (0,0,255)
        self.red = (255,0,0)
        self.yellow = (255,255,0)

        self.width = dimenions[0]
        self.height= dimenions[1]

      
        pygame.display.set_caption("Diffrential Bot")
        self.map = pygame.display.set_mode((self.width,self.height))
        self.lineMap = pygame.Surface(dimenions, pygame.SRCALPHA)
        # self.lineMap.fill(self.yellow)
        self.lastMousePose = None

    
    def writeInfo(self, vl,vr,theta):
        info  = f"VL = {vl} VR = {vr} Theta = {theta}"
        font = pygame.font.Font('freesansbold.ttf',14)
        text = font.render(info, True , self.black,self.white)
        textRect = text.get_rect()
        textRect.center = (self.width-200, self.height-100)
        self.map.blit(text,textRect)

    def update(self,event):
        if event.type == pygame.MOUSEMOTION:
            if event.buttons[0]:
               
                if(self.lastMousePose):
                    pygame.draw.line(environment.lineMap, self.black, self.lastMousePose, event.pos, 15)
                self.lastMousePose =event.pos
    
        environment.map.blit(self.lineMap,(0,0))


class Robot:

    def __init__(self, startpos, robotImg, robotDimension):
        self.width,self.height = robotDimension
        self.x = startpos[0]
        self.y = startpos[1]
        self.theta = 0
        self.vl = 150
        self.vr = 150
        self.turnSpeed = 10
        self.maxSpeed = 800
        self.image = pygame.image.load(robotImg)
        self.image = pygame.transform.scale(self.image,robotDimension)
        self.rotatedImage = self.image
        self.rect = self.rotatedImage.get_rect(center=(self.x,self.y))
        self.lineSensor = LineSensor(8,self)
        self.setPos = 0.2-0.05
        self.lastError = 0
        self.totalError= 0

    def draw(self, environment):
        environment.map.blit(self.rotatedImage,self.rect)
        self.lineSensor.draw(environment,self)
        self.lineSensor.sense(environment,self)


    def setMotorSpeed(self,ls,rs):
        self.vl=ls
        self.vr=rs

    def pid(self):


        line = self.lineSensor.readBlackLine()

        if(line==0 or line ==1):
            return (self.vl,self.vr)

        kp,kd,ki = 2000,2000,50

        error = self.setPos - line
        self.totalError+=error
        # print(error)
        pid = error*kp + kd * (error-self.lastError) +ki * self.totalError

        newVl = self.vl-pid
        newVr = self.vr+pid
        self.lastError=error
        

        return (newVl, newVr)

    def update(self,ls , rs, event = None ):
        
        if event is not None:
            if event.type == pygame.KEYDOWN:
                if event.key ==pygame.K_KP4: 
                    self.vl += self.turnSpeed
                elif event.key ==pygame.K_KP1: 
                    self.vl -= self.turnSpeed
                elif event.key ==pygame.K_KP6: 
                    self.vr += self.turnSpeed
                elif event.key ==pygame.K_KP3: 
                    self.vr -= self.turnSpeed
                elif event.key ==pygame.K_a: 
                    self.vr += self.turnSpeed
                    self.vl -= self.turnSpeed
                elif event.key ==pygame.K_d: 
                    self.vr -= self.turnSpeed
                    self.vl += self.turnSpeed

                elif event.key ==pygame.K_w: 
                    self.vr += self.turnSpeed
                    self.vl += self.turnSpeed
                
                elif event.key ==pygame.K_s: 
                    self.vr -= self.turnSpeed
                    self.vl -= self.turnSpeed

                elif event.key == pygame.K_DOWN:
                    self.setPos-=0.05
                elif event.key == pygame.K_UP:
                    self.setPos+=0.05
        
        (vl,vr) = self.pid()
        vl = min(self.maxSpeed, max(-self.maxSpeed, vl))
        vr = min(self.maxSpeed, max(-self.maxSpeed, vr))

        self.x += ((vl + self. vr)/2* math.cos(self.theta)*dt)
        self.y -= ((vl + self. vr)/2* math.sin(self.theta)*dt)

        self.x = self.x % dimension[0]
        self.y = self.y % dimension[1]
        
        self.theta+=(vr-vl)/self.width*dt
        self.theta = 0 if self.theta>=2*math.pi or self.theta<=-2*math.pi else self.theta
        self.rotatedImage=pygame.transform.rotozoom(self.image,math.degrees(self.theta-math.pi/2),1)
        self.rect = self.rotatedImage.get_rect(center=(self.x,self.y))




class LineSensor:

    def __init__(self,numSensors,robot):
        self.numSensors = numSensors
        self.sensorValues = [0]*numSensors
        self.sensorSurface =pygame.Surface((robot.width,robot.height),pygame.SRCALPHA)
        self.rect = self.sensorSurface.get_rect(center=(robot.x,robot.y))

    def sense(self, environment,robot):
        sensorPos = self.getSensorPos(robot)
        for i in range(self.numSensors):
            sx,sy = sensorPos[i]
            pygame.draw.circle(environment.map,environment.green,(sx,sy),2)
            self.sensorValues[i]=environment.lineMap.get_at((sx,sy))[3]//255
   
    def getSensorPos(self,robot):
        sensorPos=[]
        x,y = self.rect.centerx+robot.height/2+5, self.rect.centery-robot.width/2+5
        gap = robot.width/self.numSensors

        def rotate_point(cx,cy,angle,px,py):
            s = math.sin(angle)
            c = math.cos(angle)
            #translate point back to origin:
            px -= cx
            py -= cy
            # rotate point
            xnew = px * c - py * s
            ynew = px * s + py * c
            # translate point back:
            px = xnew + cx
            py = ynew + cy
            return math.floor(px),math.floor(py)
       
        #find points without rotation
        for i in range(self.numSensors):
            # sensorPos.append((x,y))
            sensorPos.append(rotate_point(self.rect.centerx,self.rect.centery,-robot.theta,x,y))
            y+=gap
            
        return sensorPos

    def draw(self,environment,robot):

        pygame.draw.rect(self.sensorSurface,environment.red,(0,0,robot.width,5))
        rectSurface = pygame.transform.rotozoom(self.sensorSurface, math.degrees(robot.theta-math.pi/2),1)
        self.rect = rectSurface.get_rect(center=(robot.x,robot.y))
        environment.map.blit(rectSurface,self.rect)

    def readBlackLine(self):
        sw=100
        weightedSum = 0
        sumOfWeights = sw
        for i in range(self.numSensors):
            weightedSum += (sw*self.sensorValues[i])
            sw+=100
            sumOfWeights+=sw
        return weightedSum/sumOfWeights


pygame.init()
start = (200,200)
dimension = (800,640)
running = True
environment = Environment(dimension)
running = True

robot = Robot(start,"car.png",(66,51))
dt=0
lastTime = pygame.time.get_ticks()
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        
        if event.type == pygame.K_d:
            environment.drawMode!=environment.drawMode

        robot.update(10,10,event)

    dt = (pygame.time.get_ticks()-lastTime)/1000
    lastTime = pygame.time.get_ticks()

    pygame.display.update()
    # pygame.display.flip()
    environment.map.fill(environment.white)
    environment.update(event)
    robot.update(10,10)
    robot.draw(environment)
    environment.writeInfo(int(robot.vl), int(robot.vr),math.floor(math.degrees(robot.theta)))
    
