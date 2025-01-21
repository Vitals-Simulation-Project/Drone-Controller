from HelperFunctions import clusterHelper
import Constants.configDrones as configDrones

# global config
MIN_CIRCLE_RADIUS_GPS = configDrones.MIN_CIRCLE_RADIUS_GPS 
CIRCLE_SPACING = configDrones.CIRCLE_SPACING

class circle:
  def __init__(self, radius, avgCenter, gpsPointList):
    self.radius = radius         
    self.avgCenter = avgCenter       # average of center in a list 
    self.gpsPointList = gpsPointList # list of centers of search radii

def calcMergeCircle(circle1, circle2):
    mergedGpsPointList = circle1.gpsPointList + circle2.gpsPointList
    count = 0 
    avgGps = [0, 0]
    
    # calculate center
    for gps in mergedGpsPointList:
        avgGps[0] += gps[0]
        avgGps[1] += gps[1]
        count += 1
    avgGps[0] /= count
    avgGps[1] /= count

    # calculate search radius
    furthest = clusterHelper.furthestCentroid(mergedGpsPointList, avgGps[0], avgGps[1])
    radius = clusterHelper.distanceForm(furthest, avgGps[0], avgGps[1]) + MIN_CIRCLE_RADIUS_GPS
    mergedCircle = circle(radius, avgGps, mergedGpsPointList)
    return mergedCircle

def isMergeNeeded(circle1, circle2):
    # calculate distance between centers
    d = ((circle1.avgCenter[0] - circle2.avgCenter[0])**2 + \
         (circle1.avgCenter[1] - circle2.avgCenter[1])**2)**0.5

    r = (circle1.radius + circle2.radius) + CIRCLE_SPACING

    if (r > d):
        return True

    # check for touching, intersecting circles, and when
    # the center of one lies on the center of another
    #if d == 2*r or d == r - r or (d < 2*r and d > r - r) \
    #    or d == r:
    #    return True
    return False

def addCircle(circleToAdd, circleList):
    for i in range(len(circleList)):
        if (isMergeNeeded(circleToAdd, circleList[i])):
            mergedCircle = calcMergeCircle(circleToAdd, circleList[i])
            circleList.pop(i)   # removes element at i
            return addCircle(mergedCircle, circleList)
    circleList.append(circleToAdd)
    return circleList

def circleGroups(centroids, r):
    intersect = []
    avgCentroids = []
    inGroup = []
    elementsPerGroup = []
    i = 0

    for centroid in centroids:
        # create circle group
        if len(intersect) == 0 or centroid not in inGroup:
            intersect.append([centroid])
            inGroup.append(centroid)    # keep track of what's in a group
            avgCentroids.append(centroid)
            elementsPerGroup.append(1)  # track the number of circles in group
        else:
            continue

        for centroidPrime in centroids:
            # skip the same element or already grouped elements
            if centroid == centroidPrime or centroidPrime in inGroup:
                continue

            # calculate distance between centers
            d = clusterHelper.distanceForm(centroid, centroidPrime[0], centroidPrime[1])

            # check for touching, intersecting circles, and when
            # the center of one lies on the center of another
            if d == 2*r or d == r - r or (d < 2*r and d > r - r) \
               or d == r:
               # add circle to the group of the
               # centroid we were comparing it to
               intersect[i].append(centroidPrime)
               inGroup.append(centroidPrime)

               # start taking an average for the ith group
               avgCentroids[i] = list(avgCentroids[i])
               (avgCentroids[i])[0] += centroidPrime[0]
               (avgCentroids[i])[1] += centroidPrime[1]
               avgCentroids[i] = tuple(avgCentroids[i])

               # update the number of elements in current group
               elementsPerGroup[i] += 1

        # update the group counter
        i += 1

    # finish the the averaging by dividing each group's
    # average centroid by the number of circles in that group
    for j in range(len(avgCentroids)):
        # divide x and y component by number of circle centers
        avgCentroids[j] = list(avgCentroids[j])
        (avgCentroids[j])[0] /= elementsPerGroup[j]
        (avgCentroids[j])[1] /= elementsPerGroup[j]
        avgCentroids[j] = tuple(avgCentroids[j])

    return intersect, avgCentroids

def pixelClustering(height, width, segRGB):
    pixCount = 0
    clusters = []

    for i in range(height):
        for j in range(width):
            if segRGB[i][j][0] <= 255 and \
               segRGB[i][j][1] <= 255 and \
               segRGB[i][j][2] <= 255 and \
               segRGB[i][j][0] >= 200 and \
               segRGB[i][j][1] >= 200 and \
               segRGB[i][j][2] >= 200:
                pixCount = pixCount + 1

                isMatch = False
                isClustered = False
                pixel = [(i, j)]

                # found heat signature pixels
                if not clusters:
                    clusters.append(pixel)
                else:
                    clusterCount = 0
                    # not empty case
                    for cluster in clusters:
                        sibilingExists = False
                        for coord in cluster:
                            # properly place the cluster among appropriate color
                            imgPixel = segRGB[i][j][0]
                            clusterPixel = segRGB[coord[0]][coord[1]][0]
                            if imgPixel != clusterPixel:
                                break
                            else:
                                isMatch = True

                            # otherwise threshold and sort the coordinate
                            if clusterHelper.dist([i, j], [coord[0], coord[1]]) < 40:
                                cluster.append(pixel[0])
                                isClustered = True
                                break
                            else:
                                # used for same heat signature for two animals
                                # but they are far apart
                                sibilingExists = clusterHelper.checkSiblingClusterExists(clusterPixel, clusterCount, segRGB, clusters)
                                if not sibilingExists:
                                    isClustered = True
                                    clusters.append(pixel)
                                break

                        clusterCount+=1
                        if isClustered:
                            break
                    # no match for non-empty list
                    if not isMatch:
                        clusters.append(pixel)
    return clusters