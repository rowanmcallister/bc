Driving data generated using a RoamingAgent from [CARLA](http://carla.org/) for behavior cloning.

### Details

We follow CARLA's Town04 figure-8 loop indefinitely ([example video](https://drive.google.com/a/berkeley.edu/file/d/15IsPqlN7dKKemcE8VbE_Afebe-xVcDRi/view?usp=sharing)),
shown from above here:
<img src="images/carla-town04.png">

### Datasets
 - 10k PNG images 48x48 in lane 1 at 20Hz: [download (33 MB)](https://drive.google.com/a/berkeley.edu/file/d/1ZShsI8a_oU7gJSHvSmk7Z58uVC9FXsgc/view?usp=sharing)
 - 100k PNG images 48x48 in lane 1 at 20Hz: [download (325 MB)](https://drive.google.com/a/berkeley.edu/file/d/1HsVe84IMfhmL5pMEJMdm19MWEDyh18Gt/view?usp=sharing)
<img src="images/carla-town04-lane1-48x48.png" width="200">

Each image has an associated action, embedded in the image's metadata:
```
# Load an image:
from PIL.PngImagePlugin import PngImageFile
im = PngImageFile("rl00001234.png")

# Actions are stored in the image's metadata:
print("Actions: %s" % im.text)
throttle = float(im.text['throttle'])  # range [0, 1]
steer = float(im.text['steer'])   # range [-1, 1]
brake = float(im.text['brake'])   # range [0, 1]
```
