# canopyMissing
C++ algorithm to estimate canopy missing of a stem tree using PCL

## Input file structure support

* .pcd 
* .ply
* .txt
* .xyz

## Output file structure .txt

* percentage_canopy_missing %

## Example
<img src="./example/scan1.png" align="center" height="400" width="720"><br>

-------------------
## Compilation
* Set "YOUR OWN" PCL Build DIR in CMakeList.txt e.g: **/opt/pcl-1.8.1/build** and save it.
* Set "YOUR OWN" OpenCV Build DIR in CMakeList.txt e.g: **/opt/opencv-4.1.0/build** and save it.
* Create a "build" folder

in the main folder:

    - cd build  
    - cmake ../
    - make
       
        	 
### Test

    cd /build/bin
    ./canopyMissing <pcd file> <output dir> 
     
