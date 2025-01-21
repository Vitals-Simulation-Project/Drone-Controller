import airsim

# Functions for calculating euclidian distance,
# calculating the distance formula, and averaging centroids
def dist(P, Q):
    return (sum((pi - qi)**2 for pi, qi in zip(P, Q)))**0.5

def distanceForm(centroid, avgCentroidsX, avgCentroidsY):
    distance = ((centroid[0] - avgCentroidsX)**2 + \
                (centroid[1] - avgCentroidsY)**2)**0.5

    return distance

# Functions to help organize clusters
def checkSiblingClusterExists(pixelRGB, currentCluster, segRGB, clusters):
    if currentCluster==len(clusters):
        return False

    for i in range(currentCluster+1, len(clusters)):
        clusterColor = segRGB[clusters[i][0][0]][clusters[i][0][1]][0]
        if(pixelRGB==clusterColor):
            return True

    return False

def furthestCentroid(centroids, avgCentroidsX, avgCentroidsY):
    far = 0
    furthestCentroid = (0, 0)
    for centroid in centroids:
        # calculate distance between center and one of the cluster centroids
        distance = distanceForm(centroid, avgCentroidsX, avgCentroidsY)

        # compare the distance to find the furthest centroid
        if distance >= far:
            furthestCentroid = centroid
            far = distance

    return furthestCentroid

def applyInfrared(client):
    # set target color in segmentation
    client.simSetSegmentationObjectID("[\w]*", 0, True);
    client.simSetSegmentationObjectID('.*?FoxMasterAi.*?', 215, True);  # fox
    client.simSetSegmentationObjectID('.*?StagMasterAi.*?', 230, True); # stag
    client.simSetSegmentationObjectID('.*?DoeMasterAi.*?', 200, True);  # doe
    client.simSetSegmentationObjectID('.*?BrianMasterAi.*?', 255, True);# brian
    client.simSetSegmentationObjectID('.*?BP_MovedChar.*?', 255, True); # moving brian mesh
    client.simSetSegmentationObjectID('.*SK_Wolf.*?', 255, True)        # white wolf
