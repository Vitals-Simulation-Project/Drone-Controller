# In settings.json first activate computer vision mode: 
# https://github.com/Microsoft/AirSim/blob/main/docs/image_apis.md#computer-vision-mode

import airsim
import cv2
import numpy as np
import setup_path 

client = airsim.VehicleClient()
client.confirmConnection()

#get segmentation image in various formats
responses = client.simGetImages([
    airsim.ImageRequest("0", airsim.ImageType.Segmentation, False, False),  #scene vision image in uncompressed RGBA array
    airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])
print('Retrieved images: ', len(responses))

#save segmentation images in various formats
for idx, response in enumerate(responses):
    filename = '../../images/py_seg_' + str(idx)

    if response.pixels_as_float:
        print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
        airsim.write_pfm((filename + '.pfm'), airsim.get_pfm_array(response))
    elif response.compress: #png format
        print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
        airsim.write_file((filename + '.png'), response.image_data_uint8)
    else: #uncompressed array - numpy demo
        print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
        img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) #get numpy array
        img_rgb = img1d.reshape(response.height, response.width, 3) #reshape array to 3 channel image array H X W X 3
        cv2.imwrite((filename + '.png'), img_rgb) # write to png

        #find unique colors
        print(np.unique(img_rgb[:,:,0], return_counts=True)) #red
        print(np.unique(img_rgb[:,:,1], return_counts=True)) #green
        print(np.unique(img_rgb[:,:,2], return_counts=True)) #blue  