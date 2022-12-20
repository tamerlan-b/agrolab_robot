# Управление роботом, заданным в URDF  

### Машина состояний
![Машина состояний](media/agrolab_fsm.png)

### Установка зависимостей  

Визуализатор машины состояний:
```bash
sudo apt install ros-noetic-smach-viewer
```

Зависимости для детектора:
* opencv
* cv_bridge

### Подготовка
1. Создать ROS-workspace
```bash  
mkdir -p agrolab_ws/src
```
2. Склонировать текущий репозиторий в папку src внутри workspace:  
```bash  
cd agrolab_ws/src
git clone https://github.com/tamerlan-b/agrolab_robot
cd ..
```
3. Собрать проект:  
```bash  
source /opt/ros/noetic/setup.bash
catkin_make
```

### Запуск системы (RViz, Gazebo, машина состояний и поиск объекта)
```bash  
source devel/setup.bash
roslaunch state_machine system.launch
```

### Запуск машины состояний
```bash  
source devel/setup.bash
rosservice call /state_machine/start_fsm "{}"
```


### Camera

Приделал камеру, приделана к оси Y, нужно проверить что с ней будет в движении, т.к. пока функционала нет
Координаты если что находятся в `urdf/agrolab.urdf` (координаты относительно центры сцены):

```
217 <origin xyz="-0.26 -0.26 0.4" rpy="0 1.57 0"/>
```

получить raw изображение с камеры можно через топик:

```
/cam_top/image_raw
```

Рядом с этим топиком можно так же найти другие топики, относящиеся к камере:

```
/cam_top/image_raw
/cam_top/parameter_descriptions
/cam_top/parameter_updates
```

При необходимости можно поменять параметры камеры, если нужно, находитя в том же `.urdf`, со строки 245 