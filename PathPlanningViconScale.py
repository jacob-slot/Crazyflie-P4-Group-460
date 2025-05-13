import numpy as np
import shapely.geometry as sg
import matplotlib.pyplot as plt

fileLocation = r"FILELOCATION HERE" #Replace with the location of your file

#Coordinates of the field polygon (In meters)

#Square
# SquareSize = 1.5
# fieldCoords = np.array([
#     [ SquareSize,  SquareSize],  
#     [-SquareSize,  SquareSize],  
#     [-SquareSize, -SquareSize],  
#     [ SquareSize, -SquareSize],  
#  ])



# #TRAPEZ
fieldCoords = np.array([
    [ 1.5,  -1.1],
    [ 1.5,   1.1],
    [-2,  1.5],
    [-2, -1.5]
])


#Square with a folded corner:
# fieldCoords = np.array([
#     [ 0.0,  1.5],  
#     [-1.5,  0.75],  
#     [-1.5, -1.5],  
#     [ 1.5, -1.5],  
#     [ 1.5,  1.5]   
# ])

#Rectangle
# fieldCoords = np.array([
#     [ -1.5,  1],
#     [ -1.5, -1],
#     [  1.5, -1],
#     [1.5,    1]
# ])

#Scaling factor for the Vicon system
viconScale = 30

def generatePath(fieldCoords, cameraLength = 27/viconScale,cameraWidth = 15.7/viconScale, overlap = 0.0385*2):
    # Create a polygon from the field coordinates
    fieldPolygon = sg.Polygon(fieldCoords)

    #find field bounding box
    minX, minY, maxX, maxY = fieldPolygon.bounds

    #Generate halfLength and halfWidth based on camera length and width with overlap
    halfLength= (cameraLength*(1-overlap))/2
    halfWidth = (cameraWidth*(1-overlap))/2


    #Generate x values with 3.85% overlap
    xValues = np.arange(minX+halfLength, maxX, cameraLength*(1-overlap))

    #add final x value at maxX-halfLength
    #xValues = np.append(xValues, maxX-halfLength)
    

    #Initial waypoints list and direction variable
    waypoints = []
    direction = 1

    #find waypoints
    for x in xValues:
        #inital values for outOfBounds variables
        outOfBoundsLeft = False
        outOfBoundsRight = False


        #Create vertical lines for intersection checks
        verticalLine = sg.LineString([(x, minY-1), (x, maxY+1)])
        verticalLineLeft = sg.LineString([(x-halfLength, minY-1), (x-halfLength, maxY+1)])
        verticalLineRight = sg.LineString([(x+halfLength, minY-1), (x+halfLength, maxY+1)])


        #find intersections
        intersectionLeft = fieldPolygon.intersection(verticalLineLeft)
        intersectionRight = fieldPolygon.intersection(verticalLineRight)
        intersectionMid = fieldPolygon.intersection(verticalLine)


        #find amount of intersection points, and convert to 2 point variables, if more than 2 intersections are found
        if isinstance(intersectionLeft, sg.MultiLineString):
            lenLeft = sum(len(line.coords) for line in intersectionLeft.geoms)
            intersectionLeft = sg.LineString([pt for line in intersectionLeft.geoms for pt in line.coords])
            intersectionLeft = sg.LineString([(intersectionLeft.xy[0][0],intersectionLeft.xy[1][0]),(intersectionLeft.xy[0][-1],intersectionLeft.xy[1][-1])])
        else:
            lenLeft = len(intersectionLeft.xy[0])
        
        if isinstance(intersectionRight, sg.MultiLineString):
            lenRight = sum(len(line.coords) for line in intersectionRight.geoms)
            intersectionRight = sg.LineString([pt for line in intersectionRight.geoms for pt in line.coords])
            intersectionRight = sg.LineString([(intersectionRight.xy[0][0],intersectionRight.xy[1][0]),(intersectionRight.xy[0][-1],intersectionRight.xy[1][-1])])
        else:
            lenRight = len(intersectionRight.xy[0])

        if isinstance(intersectionMid, sg.MultiLineString):
            intersectionMid = sg.LineString([pt for line in intersectionMid.geoms for pt in line.coords])
            intersectionMid = sg.LineString([(intersectionMid.xy[0][0],intersectionMid.xy[1][0]),(intersectionMid.xy[0][-1],intersectionMid.xy[1][-1])])
        

        #Check for only one point of intersection. If true create new vertical line
        if lenLeft<=1:
            intersectionLeft = fieldPolygon.intersection(verticalLine)
            outOfBoundsLeft = True
        if lenRight<=1:
            intersectionRight = fieldPolygon.intersection(verticalLine)
            outOfBoundsRight = True


        #Check which intersection has the highest value
        if intersectionMid.xy[1][1] > intersectionLeft.xy[1][1] and intersectionMid.xy[1][1] > intersectionRight.xy[1][1]:
            #mid higher
            waypoints.append((intersectionMid.xy[0][1],intersectionMid.xy[1][1]-halfWidth))
        elif intersectionLeft.xy[1][1] > intersectionRight.xy[1][1] and intersectionLeft.xy[1][1] > intersectionMid.xy[1][1]:
            #left side higher
            if outOfBoundsLeft:
                waypoints.append((intersectionLeft.xy[0][1],intersectionLeft.xy[1][1]-halfWidth))
            else:
                waypoints.append((intersectionLeft.xy[0][1]+halfLength,intersectionLeft.xy[1][1]-halfWidth))
        else:
            #right side higher
            if outOfBoundsRight:
                waypoints.append((intersectionRight.xy[0][1],intersectionRight.xy[1][1]-halfWidth))
            else:
                waypoints.append((intersectionRight.xy[0][1]-halfLength,intersectionRight.xy[1][1]-halfWidth))


        #check which intersection has the lowest value
        if intersectionMid.xy[1][0] < intersectionLeft.xy[1][0] and intersectionMid.xy[1][0] < intersectionRight.xy[1][0]:
            #mid lower
            waypoints.append((intersectionMid.xy[0][0],intersectionMid.xy[1][0]+halfWidth))
        elif intersectionLeft.xy[1][0] < intersectionRight.xy[1][0] and intersectionLeft.xy[1][0] < intersectionMid.xy[1][0]:
            #left side lower
            if outOfBoundsLeft:
                waypoints.append((intersectionLeft.xy[0][0],intersectionLeft.xy[1][0]+halfWidth))
            else:
                waypoints.append((intersectionLeft.xy[0][0]+halfLength,intersectionLeft.xy[1][0]+halfWidth))
        else:
            #right side lower
            if outOfBoundsRight:
                waypoints.append((intersectionRight.xy[0][0],intersectionRight.xy[1][0]+halfWidth))
            else:
                waypoints.append((intersectionRight.xy[0][0]-halfLength,intersectionRight.xy[1][0]+halfWidth))
        

        #if direction=1 swap two newest added waypoints
        if direction == 1:
            waypoints[-2], waypoints[-1] = waypoints[-1], waypoints[-2]   
        
        direction *= -1
        
    
    return waypoints, fieldPolygon

waypoints, fieldPolygon = generatePath(fieldCoords)

#Plot polygon and waypoints
x, y = zip(*waypoints)
fig, ax = plt.subplots()
ax.plot(*fieldPolygon.exterior.xy, 'k-', label="Field Boundary")
ax.plot(x, y, 'ro-', label="Flight Path")
ax.set_aspect('equal')
plt.legend()
plt.show()

# Save waypoints to a file

height = 1
with open(fileLocation, 'w') as f:
    for waypoint in waypoints:
        f.write(f"{waypoint[0]}, {waypoint[1]},{height}\n")

#Create a new file. For every waypoint add the four corner values of the camera
#with the height of the camera
with open(fileLocation, 'w') as f:
    for waypoint in waypoints:
        halfLength= (27/viconScale)/2
        halfWidth = (15.7/viconScale)/2

        f.write(f"{waypoint[0]-halfLength}, {waypoint[1]-halfWidth},{height}\n")
        f.write(f"{waypoint[0]+halfLength}, {waypoint[1]-halfWidth},{height}\n")
        f.write(f"{waypoint[0]+halfLength}, {waypoint[1]+halfWidth},{height}\n")
        f.write(f"{waypoint[0]-halfLength}, {waypoint[1]+halfWidth},{height}\n")
