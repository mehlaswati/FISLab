from PIL import Image,ImageDraw
from PriorityQueue import PriorityQueue
import math
import Winter
import Spring
import fall
"""
Lab1_swati.py 
Author: FNU Swati
"""

# defaultSpeed = 2 metre per second
class map:

  __slots__ = 'terrain_dict','openLand','roughMeadow','easyMovementForest','slowRunForest','walkForest', 'impassibleVegtn','lakeSwampMarsh'\
                ,'pavedRoad','foothPath','outOfBounds','pixelsList','elevationList','horizontalDistance','verticalDistance','diagonalDistance'\
                ,'listOfControlsToVisit','imageSize','visited','winterIceWater','mud','test','speedMatrix','fall'




  def __init__(self,controlFileName,imageName, season):
      self.openLand = (248,148,18)
      self.roughMeadow = (255,192,0)
      self.easyMovementForest = (255,255,255)
      self.slowRunForest = (2,208,60)
      self.walkForest = (2,136,40)
      self.impassibleVegtn = (5,73,24)
      self.lakeSwampMarsh = (0,0,255)
      self.pavedRoad = (71,51,3)
      self.foothPath = (0,0,0)
      self.outOfBounds = (205,0,101)
      self.winterIceWater = (129,255,254)
      self.mud = (146,125,102)
      self.elevationList = []
      self.horizontalDistance = 10.29
      self.verticalDistance = 7.55
      self.fall = (215,141,115)
      self.speedMatrix = []
      self.test = []

      self.diagonalDistance = math.sqrt((self.horizontalDistance*self.horizontalDistance)+ (self.verticalDistance*self.verticalDistance))
      self.imageSize = 0
      self.pixelsList = []
      self.readImage(imageName)
      self.getElevation()


      if season == 'summer':


          self.terrain_dict = {self.openLand: 18, self.roughMeadow: 7, self.easyMovementForest: 15,
                               self.slowRunForest: 14, self.walkForest: 12, self.impassibleVegtn: 1,
                               self.lakeSwampMarsh: 1,
                               self.pavedRoad: 20, self.foothPath: 19, self.outOfBounds: 0}

      elif season == 'fall':
          self.terrain_dict = {self.openLand: 18, self.roughMeadow: 7, self.easyMovementForest: 15,
                               self.slowRunForest: 14, self.walkForest: 12, self.impassibleVegtn: 1,
                               self.lakeSwampMarsh: 1,self.fall: 11,
                               self.pavedRoad: 20, self.foothPath: 19, self.outOfBounds: 0}

      elif season == 'winter':

          self.terrain_dict = {self.openLand: 18, self.roughMeadow: 7, self.easyMovementForest: 15,
                               self.slowRunForest: 14, self.walkForest: 12, self.impassibleVegtn: 1,
                               self.lakeSwampMarsh: 1, self.winterIceWater : 4,
                               self.pavedRoad: 20, self.foothPath: 19, self.outOfBounds: 0}


      elif season == 'spring':
          self.terrain_dict = {self.openLand: 18, self.roughMeadow: 4, self.easyMovementForest: 10,
                               self.slowRunForest: 15, self.walkForest: 13, self.impassibleVegtn: 2,
                               self.lakeSwampMarsh: 1,self.mud : 1,
                               self.pavedRoad: 20, self.foothPath: 19, self.outOfBounds: 0}


      self.visited = []

      self.listOfControlsToVisit = []

      with open(controlFileName) as pathFile:
          self.listOfControlsToVisit.append([line.split() for line in pathFile])


  def calculateSpeed(self,imageObject):

      rgbImage = imageObject.convert('RGB')
      pixval = rgbImage.load()
      width, height = rgbImage.size
      for x in range(width):
          localSpeedList = []
          for y in range(height):
              if (self.pixelsList[x][y] == self.openLand):
                  localSpeed = self.terrain_dict[self.openLand]
              elif (self.pixelsList[x][y] == self.roughMeadow):
                  localSpeed = self.terrain_dict[self.roughMeadow]
              elif (self.pixelsList[x][y] == self.easyMovementForest):
                  localSpeed = self.terrain_dict[self.easyMovementForest]
              elif (self.pixelsList[x][y] == self.slowRunForest):
                  localSpeed = self.terrain_dict[self.slowRunForest]
              elif (self.pixelsList[x][y] == self.walkForest):
                  localSpeed = self.terrain_dict[self.walkForest]
              elif (self.pixelsList[x][y] == self.impassibleVegtn):
                  localSpeed = self.terrain_dict[self.impassibleVegtn]
              elif (self.pixelsList[x][y] == self.lakeSwampMarsh):
                  localSpeed = self.terrain_dict[self.lakeSwampMarsh]
              elif (self.pixelsList[x][y] == self.foothPath):
                  localSpeed = self.terrain_dict[self.foothPath]
              elif (self.pixelsList[x][y] == self.outOfBounds):
                  localSpeed = self.terrain_dict[self.outOfBounds]
              # elif (self.pixelsList[x][y] == self.winterIceWater):
              #     localSpeed = self.terrain_dict[self.winterIceWater]
              # elif (self.pixelsList[x][y] == self.mud):
              #     localSpeed = self.terrain_dict[self.mud]
              localSpeedList.append(localSpeed)
          self.speedMatrix.append(localSpeedList)


  def readImage(self,imageObject):

      """ This function reads an image into a 2D matrix called pixelList
      :param imageObject : The image object to be read.
      :return 2D matrix of pixels and Size of image"""


      rgbImage = imageObject.convert('RGB')
      pixval = rgbImage.load()

      width, height = rgbImage.size

      for x in range(width):
          all_nodes = []
          for y in range(height):
              all_nodes.append(pixval[x,y])
          self.pixelsList.append(all_nodes)
      # print('pixel',len(self.pixelsList))
      self.imageSize = rgbImage.size
      return self.pixelsList, rgbImage.size

  def getNeighbor(self, node, imageSize):

      """ This function finds the neighbor of a given node in all the eight directions.
      :param node : the node to find neighbor of
      :param imageSize: the size of image
      :return the list of all the neighbors of current node

      """

      col, rows = imageSize
      dirs = [[0, 1], [0, -1], [-1, 0], [1, 0],[-1,-1],[-1,1],[1,-1],[1,1]]
      #
      result = []
      for dir in dirs:
          horizontal = int(node[0]) + dir[0]
          vertical = int(node[1]) + dir[1]
          if horizontal < col and horizontal >= 0 and vertical < rows and vertical >= 0 and self.pixelsList[horizontal][vertical] != (205,0,101)   :
              if self.pixelsList[horizontal][vertical] != self.outOfBounds  :

                    result.append((horizontal, vertical))
      return result

  def getElevation(self):
      """ This function creates the elevation matrix
      :return None

      """

      with open("elevations") as textFile:
          for line in textFile:
              counter = 0
              elevationListPerItem = []
              for item in line.split():
                  # print(item)
                  counter +=1
                  if counter <= 395:
                    elevationListPerItem.append(item)
              self.elevationList.append(elevationListPerItem)



  def heuristic(self,current,next):
      """ This function creates heuristic
      :param current : the node to find neighbor of
      :param next: Goal control
      :return the value of heuristic between two nodes

      """

      currentX = int(current[0])
      currentY = int(current[1])
      nextX = int(next[0])
      nextY = int(next[1])



      dx = abs(currentX - nextX) * self.horizontalDistance
      dy = abs(currentY - nextY) * self.verticalDistance



      heuristics = math.sqrt((dx**2)+(dy**2))

      # print('hn', heuristics/20)
      return heuristics/20



  def drawPath(self, im, list):
      """ This function draws a path on image from a given list of coordinates
      :param im : object of image to draw path on, and displays the image
      :param list: list of coordinates
      :return None

      """
      draw = ImageDraw.Draw(im)
      draw.line(list, fill=(255, 0, 0), width=2)
      im.show()

  def costBetweenTwoNodes(self, current, next,season):

      """ This function calculates the path cost  between two adjacent nodes
      :param current : object of image to draw path on, and displays the image
      :param next: next coordinates
      :param season: season
      :return path cost in terms of time.

      """

      currentX = int(current[0])
      currentY = int(current[1])
      nextX = int(next[0])
      nextY = int(next[1])
      # print('here',self.pixelsList[nextX][nextY])
      speed = self.terrain_dict[self.pixelsList[nextX][nextY]]

      elevationofCurrent = float(self.elevationList[currentY][currentX])
      elevationOfNext = float(self.elevationList[nextY][nextX])
      differenceInElevation = elevationOfNext - elevationofCurrent
      # print('diff in el',differenceInElevation)
      if currentY == nextY -1 or currentY == nextY +1 and nextX == currentX:
          distance = self.verticalDistance
      elif currentX == nextX -1 or currentX == nextX +1 and nextY == currentY:
          distance = self.horizontalDistance
      else:
          distance = self.diagonalDistance

      distanceF = math.sqrt((distance ** 2) + (differenceInElevation ** 2))
      if (elevationOfNext) == float(elevationofCurrent):
          speed = speed
      elif   differenceInElevation > 0:
           speed = speed - (differenceInElevation/100)*speed
      elif differenceInElevation <0:
          speed = speed - (differenceInElevation/100)*speed
      totalDistance =   distanceF/speed
      # print('gn',time)
      return totalDistance


  def a_star_search(self,start, goal, imageSize,season):

      """ This function performs A* search
      :param start : start coordinate
      :param goal: goal coordinates
      :param imageSize: Size of image
      :param season: Season
      :return final path between nodes

      """



      frontier = PriorityQueue()
      frontier.put(start, 0)
      came_from = {}
      cost_so_far = {}
      came_from[start] = None
      cost_so_far[start] = 0
      # test = []
      priorityDict = {}
      priorityDict[start] = self.heuristic(start,goal)
      while not frontier.empty():

          current = frontier.get()

          path = []
          if current == goal:

              current = goal
              while current != start:  # loop backwards from end to start
                  path.insert(0, current)  # prepend current to the path list
                  current = came_from[current]  # move to the predecessor
              path.insert(0, start)
              # print(path)
              break


          for next in self.getNeighbor(current, imageSize):
              new_cost = cost_so_far[current] + self.costBetweenTwoNodes(current, next,season)
              priority = self.heuristic(next, goal) + new_cost

              if next not in cost_so_far or priority < priorityDict[next] :
                  # print('next',next)
                  cost_so_far[next] = new_cost

                  priorityDict[next] = priority
                  frontier.put(next, priority)
                  # print(next)
                  came_from[next] = current
      return path

  def getFinalPath(self,imageObject,season):

      """ This function get
      :param start : start coordinate
      :param goal: goal coordinates
      :param imageSize: Size of image
      :param season: Season
      :return final path between nodes

      """


      finalPath = []
      controls = self.listOfControlsToVisit
      for i in range(0, len(controls[0]) - 1):
          #
          start = controls[0][i]
          end = controls[0][i + 1]

          startTouple = (int(start[0]), int(start[1]))
          endTouple = (int(end[0]), int(end[1]))


          finalPath += self.a_star_search(startTouple, endTouple, self.imageSize,season)

      self.drawPath(imageObject, finalPath)
      imageObject.save('result'+season+'.png')
      # print(len(finalPath))
      return finalPath



  def getTotalDistainceOfTrack(self,finalPath):
      """ This function get
      :param finalPath : list of final path
      :return Total distance among control points
       """
      totalDistance = 0
      for i in range(len(finalPath) - 1):

          current = finalPath[i]
          next = finalPath[i + 1]
          currentX = int(current[0])
          currentY = int(current[1])
          nextX = int(next[0])
          nextY = int(next[1])

          if nextX == currentX - 1 or nextX == currentX + 1 and nextY == currentY:
              totalDistance += self.horizontalDistance
          elif nextY == currentY - 1 or nextY == currentY + 1 and nextX == currentX:
              totalDistance += self.verticalDistance
          else:
              totalDistance += self.diagonalDistance

      return totalDistance







if __name__ == '__main__':

    season = input('Enter The Season(summer/fall/spring/winter) ')
    controlFile = input('Enter the control file(brown/white/red) ')
    imageFile = input('Enter Image path(Image path)' )
    # season = 'spring'
    # controlFile = 'brown'
    # classObj = map(controlFile,'terrain.png',season)
    # test = classObj.readImage('terrain.png')

    if season == 'summer':
        image1 = Image.open(imageFile)
        classObj = map(controlFile, image1, season)
        finalPath = classObj.getFinalPath(image1,season)
        print('Total Distance',classObj.getTotalDistainceOfTrack(finalPath))

    elif season == 'winter':
        updatedImage = Winter.updateImageForWinterSeason(imageFile)

        classObj = map(controlFile, updatedImage, season)
        finalPath = classObj.getFinalPath(updatedImage,season)

        print('Total Distance',classObj.getTotalDistainceOfTrack(finalPath))

    elif season == 'spring':
        updatedImage = Spring.updateImageForSpringSeason(imageFile,controlFile)
        classObj = map(controlFile, updatedImage, season)
        finalPath = classObj.getFinalPath(updatedImage,season)
        print('Total Distance', classObj.getTotalDistainceOfTrack(finalPath))

    elif season == 'fall':
        updatedImage = fall.updateImageForFallSeason(imageFile)
        classObj = map(controlFile, updatedImage, season)
        finalPath = classObj.getFinalPath(updatedImage, season)

        print('Total Distance', classObj.getTotalDistainceOfTrack(finalPath))



