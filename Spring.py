from PIL import Image,ImageDraw
import Winter
import lab1

def getNeighbors(node,imageSize,pixval):
    """ This function finds the neighbor of a given node in all the four directions.
    :param node : the node to find neighbor of
    :param imageSize: the size of image
    :return the list of all the neighbors of current node"""

    col, rows = imageSize
    dirs = [[1, 0], [0, 1], [-1, 0], [0, -1]]
    result = []
    for dir in dirs:
        horizontal = int(node[0]) + dir[0]
        vertical = int(node[1]) + dir[1]

        if horizontal < col and horizontal >= 0 and vertical < rows and vertical >= 0 and pixval[horizontal,vertical] != (205,0,101)   :
            result.append((horizontal, vertical))
    return result


def getAllNodes(pixval,imageSize):
    """ This function finds the edges of water(blue) pixel.
    :param pixval : 2D matrix of pixels value
    :param imageSize: the size of image
    :return the list of all the water boundaries"""

    waterBoundariesList = []
    width, height = imageSize
    all_nodes = []
    for x in range(width-1):
        for y in range(height-1):
            all_nodes.append([x, y])
    for node in all_nodes:
        neighborList = getNeighbors(node,imageSize,pixval)
        # print('node,',node)
        for neighbor in neighborList:
            if pixval[node[0], node[1]] != pixval[neighbor[0], neighbor[1]]:
                if (pixval[node[0],node[1]] == (0,0,255) and  pixval[neighbor[0],neighbor[1]] != (0,0,255)):
                    if neighbor not in waterBoundariesList:
                        waterBoundariesList.append(neighbor)


    return waterBoundariesList



def updatePixel(image,resultList,pixelsColor):
    """ This function finds the neighbor of a given node in all the eight directions.
    :param pixval : 2D matrix of pixels value
    :param imageSize: the size of image
    :return the list of all the water boundaries"""

    img = Image.open(image)  # create a new black image
    pixels = img.load()  # create the pixel map
    # print((resultList[0]))
    for pathCoordinates in resultList:
        x = pathCoordinates[0]
        y = pathCoordinates[1]
        pixels[x, y] = (pixelsColor)
    return img




def getFifteenPixelFromWater(waterBoundaryList,image,imageSize,pixelsList):
    """ This function finds 15 pixels outside of water boundary also including the elevation
    constraint as described in question
    :param waterBoundaryList : list of boundary of water
    :param image: imageObject
    :param imageSize: imageObject
    :param pixelsList: List of pixels
    :return final list of pixel including values till 15 depth out of water and elevation less than 1"""


    classObj = lab1.map("brown", image, 'spring')
    elevationList = classObj.elevationList
    ff = []

    elevationdict = {}
    for points in waterBoundaryList:

        finalIcePoints = {}
        start = points
        # implementing BFS from here
        queue = []
        start_elevation = float(elevationList[start[1]][start[0]])
        queue.append((start, 0,start_elevation))
        predecessors = {}
        predecessors[start] = None
        currentDepth = 0

        while len(queue) > 0:

            current = queue.pop(0)
            currentCoordinate = current[0]
            currentDepth = current[1]
            totalElevation = current[2]
            elvation = float(elevationList[currentCoordinate[1]][currentCoordinate[0]])
            if currentDepth == 15 :
                break

            neighborList = getNeighbors(currentCoordinate, imageSize,pixelsList)

            # neighbor_elevation - elvation <= 1
            if totalElevation - start_elevation <1:
                for neighbor in neighborList:
                    if neighbor not in finalIcePoints:
                        neighbor_elevation = float(elevationList[neighbor[1]][neighbor[0]])

                        if  pixelsList[neighbor[0],neighbor[1]] != (0, 0, 255) and neighbor_elevation-elvation <1 :
                            elevationDifference = float(neighbor_elevation - elvation  )

                            totalElevation += elevationDifference



                            queue.append((neighbor, currentDepth + 1,totalElevation ))
                            finalIcePoints[neighbor] = 1
                            ff.append(neighbor)

    return ff
 # print('neigbor', neighbor)




def updateImageForSpringSeason(imageName,controlFile):
    """ This function open the image, read it, find boundaries and relevant neigbors and pixels
    to update the image of color, send the updated image back to the main file
    :param imageName : Name of image
    :param controlFile: List of controls to visit
    :return updated image"""

    image1 = Image.open(imageName)
    pixelList, imageSize = Winter.readImage(imageName)
    waterBoundaryList = getAllNodes(pixelList, imageSize)
    updatedMudPoints = getFifteenPixelFromWater(waterBoundaryList,image1, imageSize, pixelList)
    img = updatePixel(imageName,updatedMudPoints,(146,125,102))

    return img



if __name__ == '__main__':
    image1 = Image.open('terrain.png')
    pixval, imageSize = Winter.readImage('terrain.png')
    waterBoundaryList = getAllNodes(pixval, imageSize)
    getFifteenPixelFromWater( waterBoundaryList, image1, imageSize, pixval)