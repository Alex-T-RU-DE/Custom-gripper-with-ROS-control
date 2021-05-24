# Кастомный захват Kuka Youbot с поддержкой ROS
Language: [English](https://github.com/Alex-T-RU-DE/Custom-gripper-for-youbot-with-ROS-control/blob/main/README.md), [Русский](https://github.com/Alex-T-RU-DE/Custom-gripper-for-youbot-with-ROS-control/blob/main/README.ru.md) 

Этот проект описывает как сделать и интегрировать новый захват котролируемый ROSом для Kuka youbot. Этот репозиторий описывает последовательность действий, для создания и подключения нового захвата к мобильной платформы Kuka Youbot. 
Для реализации контроля сервоприводов используется плата Arduino, поддерживающая ROS сервисы и соединяющаяся с компьютером робота и сервоприводами через UART. 

## Необходимые компоненты

- Два сервопривода
- Распечатанный на 3D принтере и собраный захват. Модели и информацию можно найти [здесь](https://www.thingiverse.com/thing:4764063).
- Arduino mega/[pro](https://www.amazon.de/ARCELI-Arduino-Mega-ATmega2560-CH340G-Elektronik/dp/B07MQ1J9MR/ref=sr_1_13?dchild=1&keywords=arduino+pro&qid=1613692717&sr=8-13)
- Провода
- [Трансформаторы напряжения](https://www.amazon.de/LAOMAO-Wandler-einstellbar-Spannungswandler-Converter/dp/B00HV4EPG8/ref=asc_df_B00HV4EPG8/?tag=googshopde-21&linkCode=df0&hvadid=231941675984&hvpos=&hvnetw=g&hvrand=3852759402861473550&hvpone=&hvptwo=&hvqmt=&hvdev=c&hvdvcmdl=&hvlocint=&hvlocphy=9068552&hvtargid=pla-420005320986&psc=1&th=1&psc=1)


## Программа:

Перед тем как начать, вам необходимо установить Arduino IDE.

1. Копируйте "youbot_gripper" в вашу папку catkin_ws/src.

2. Откройте ваш catking_ws и выполните компиляцию "Youbot gripper" с помощью catkin_make.

3. Установите пакет ROSserial:
 ```
sudo apt-get install ros-<your_ros_version>-rosserial-arduino
sudo apt-get install ros-<your_ros_version>-rosserial
 ```
4. откройте <расположение вашей папки Arduino>/Arduino/library в терминале и выполните следующую команду:
```
rosrun rosserial_arduino make_libraries.py .
```
Эта команда автоматически конвертирует и копирует все ваши новые ROS-сообщения, сервисы и пакеты (включая пакет Youbot gripper) в библиотеки Arduino.

5. Загрузите файл "Gripper_service_server.ino" и расположите его в вашей папке Arduino.

6. Сначала попробуйте выполнить компиляцию программы без ее загрузки. Если она сработала - загрузите программу в плату Arduino и переходите к шагу 7. Если вы получаете следующую ошибку:

```
#include <cstring>
    ^~~~~~~~~
compilation terminated. 
 ```
то вы должны сменить файл "**msg.h**" в вашей директории `~/Arduino/libraries/ros_lib/ros2`, которая была сгенерирована командой ```rosrun rosserial_arduino make_libraries.py .``` . Сделать нужно это следующим образом: заходим `скетч>подключить библиотеку>управлять библиотеками` в Arduino IDE, находим и скачиваем через поиск библиотеку "*rosserial lib*" (0.7.8 version), заходим в `~/Rosserial_Arduino_Library/src/ros/`, копируем файл "**msg.h**" и заменяем на него файл с таким же именем в `~/Arduino/libraries/ros_lib/ros2`. После этого библиотеку `Rosserial_Arduino_Library` можно удалить.

Для того, чтобы запустить програму с грипером на вашем устройстве, к которому он подключен, вам нужно выполнить следующую команду:
   
7. Для того, чтобы запустить програму с грипером на вашем устройстве, к которому он подключен, вам нужно выполнить следующую команду:
```
rosrun rosserial_python serial_node.py /dev/tty_Ваш_порт_Ардуино
```
Пример:  
```
rosrun rosserial_python serial_node.py /dev/ttyUSB0
```
Эта команда запустит сервер на вашем Arduino и предоставит вам сервис для захвата. Вы можете попробовать использовать этот сервис, запустив простую программу с клиентом следующей командой:
```
rosrun youbot_gripper youbot_gripper_client номер_инструкции 
```
Где ```номер_инструкции``` должен быть заменен согласно значениям, перечисленных ниже.         
                                 
## Инструкции для сервиса

Для того, чтобы использовать команды для данного захвата, вы можете использовать следующие значения чтобы достичь желаемого результата:


```0-100``` - открыть захват на определенный процент

```1000-1380``` задать позицию первого сервопривода в диапазоне 0-380

```2000-2380``` задать позицию второго сервопривода в диапазоне 0-380

## Реакция на команды

Для того, чтобы мониторить успешность захвата, сервис возвращает позитивную или негативную реакцию, основанную на конечном количестве момента в захвате. Позитивный ответ будет отправлен в следующих случаях:

- обьект был захвачен манипулятором
- команда для правого пальца была успешно отправлена
- команда для левого пальца была успешно отправлена

## Интеграция с рукой-манипулятором в Gazebo 

Демо: https://youtu.be/vPzOUe2N2ss

Если у вас уже есть установленный пакет с симуляцией юбота то для внедрения захвата и его функционала в симулятор Gazebo вам нужно сделать следующие действия:

1. Копировать  файл `gripper_controller.launch` и расположить его в `~catkin_ws/scr/youbot_simulation/youbot_gazebo_control/launch/`.
2. Копировать файл `gripper_controller.yaml` и  `gripper_controller_1.yaml` в вашу папку `~catkin_ws/scr/youbot_simulation/youbot_gazebo_control/config`
3. Заменить ваши папки `~catkin_ws/scr/youbot_description/urdf/youbot_gripper` и `~catkin_ws/scr/youbot_description/meshes/youbot_gripper` на такие же папки из этого репозитория.

Или же вы просто можете загрузить целиком папки `youbot_description` и `youbot_simulation` в ваш `catkin_ws`.

После выполнения данных шагов вы можете запустить симуляцию с помощью следующей команды:
```
roslaunch youbot_gazebo_robot youbot.launch
``` 
Иногда, из-за разности версий появляются проблемы и ошибки, и симуляцию нужно перезапустить 2-3 раза для того, чтобы корректно загрузить контроллеры. 
Если ваша симуляция успешно загрузилась, вы можете проверить функционирование пальцев захвата с помощью следующих команд:

```
rostopic pub -1 /gripper/gripper_controller_1/command std_msgs/Float64 "data: -1.5" 
rostopic pub -1 /gripper/gripper_controller/command std_msgs/Float64 "data: 1.5"
``` 
