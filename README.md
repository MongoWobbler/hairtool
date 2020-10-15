# Hair Tool
Creates a hair curve that drives selected controls in Maya. Useful for animating secondary and follow through motion.  
[Watch a video explaining node use](https://youtu.be/nioJpRmV4S4), or read below.  
[![VIDEO](https://media.giphy.com/media/5nMPguZsJTzY5af2kt/giphy.gif)](https://youtu.be/nioJpRmV4S4 "Hair Tool")

To run:  
1. Place hairy.py in a maya script directory.
2. Run the following in python
```
import hairy
hairy.HairTool()
```
3. GUI should pop up.
4. Select controls that you want to be driven by hair curve and press "Assign Controls"
5. Select the parent of said controls, and press "Assign Parent".
6. Press "Make Hair Curve" button
7. Mess with the all of the hair system attributes until you get the result you want.
8. Press "Bake Motion" to transfer the animation onto the controls.
