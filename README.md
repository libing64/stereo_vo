# 1. stereo vo


## 1.1 steps
* get image sequence
* feature detection
* feature matching or tracking
* motion estimation(3d-2d correspondence)
* Local Optimiation(Bundle adjustment)


![kitti00](https://github.com/libing64/stereo_vo/blob/master/image/kitti00_stereo_vo.png) 

## 1.2 Building
```
cd catkin_ws/src
git clone git@github.com:libing64/stereo_vo.git
cd ..
catkin_make -DCATKIN_WHITELIST_PACKAGES="stereo_vo"
```
## 1.3 Running with kitti dataset
modify the dataset_folder in stereo_vo_kitti.launch 
```
soure devel/setup.bash
roslaunch stereo_vo stereo_vo_kitti.launch
```

## 1.5 Test Environment
Ubuntu 20.04 + ros noetic


# 2. TODO
## 2.1 How to make the estimator more robust and accurate?
- [ ] local sparse bundle adjustment
- [ ] record screen to gif
- [ ] mapping
- [ ] fusing imu 