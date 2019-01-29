## Computer Vision and A.I. - VantTEC 

# Install required modules

Tip: When you're ready to deploy the application to other computers, you can create a requirements.txt file with the command 
```bash
pip freeze > requirements.txt 
```
(pip3 on macOS/Linux). The requirements file describes the packages you've installed in your virtual environment. With only this file, you or other developers can restore those packages using
```bash
pip install -r requirements.txt 
```
(or, again, pip3 on macOS/Linux). **By using a requirements file, you need not commit the virtual environment itself to source control**.

From: [MicrosoftVSCode Docs](https://code.visualstudio.com/docs/python/environments#_manually-specify-an-interpreter)

# Color detection

Provided a cropped image with a buoy it prints the most probable color from the required list of colors using the HSV color space. It also provides a histogram and pictures both in BGR and HSV during execution.

The usage is as follows:
```bash
python color-detection.py -i test-image.jpg
```