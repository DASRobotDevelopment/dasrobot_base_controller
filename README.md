# 🛞 **fourbox_odometry** - Mecanum Одометрия для Fourbox

[ [

## 📖 Описание

ROS2 пакет для **Mecanum одометрии** 4-колёсного робота Fourbox.  
**ЧИСТАЯ Δticks за Δt** → **nav_msgs/Odometry** + **TF** для Nav2.

```
FL(0) ---- FR(1)    ← Front
 |         |
RL(2) ---- RR(3)    ← Rear
```

**Левая сторона**: FL+RL → усреднить  
**Правая сторона**: FR+RR → усреднить  
**Δθ** = `(right-left)/wheel_base_x * theta_scale(0.5)`

## 🚀 Быстрый старт

```bash
# Установка
colcon build --packages-select fourbox_odometry
source install/setup.bash

# Запуск
ros2 launch fourbox_odometry fourbox_odometry.launch.py

# Teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

## 🗺️ Topics

| Topic | Тип | Частота | Описание |
|-------|-----|---------|----------|
| `/odom` | `nav_msgs/Odometry` | 100Гц | Поза + скорости |
| `/tf` | `tf2_msgs/TFMessage` | 100Гц | `odom → base_link` |
| `/encoder_ticks` | `std_msgs/Int32MultiArray` | 100Гц | Raw тики [FL,FR,RL,RR] |

## ⚙️ Параметры

```yaml
fourbox_odometry_node:
  ros__parameters:
    wheel_radius: 0.1           # Радиус колеса (м)
    ticks_per_rev: 988          # Энкодер PPR
    wheel_base_x: 0.4           # Расстояние между сторонами (м)
    theta_scale: 0.5            # Коррекция Δθ (2x быстрее → 0.5)
    odom_frame: "odom"
    base_frame: "base_link"
```

## 🔧 Ноды

### `fourbox_odometry_node`
- **Чтение**: Serial `/dev/ttyUSB0` → тики энкодеров
- **Вычисление**: Mecanum Δθ + Trapezoidal integration
- **Публикация**: `/odom`, `/tf`, `/encoder_ticks`
- **Частота**: 100Гц (10мс Δt)

## 🧪 Тестирование

```bash
# 1. Запуск
ros2 launch fourbox_odometry fourbox_odometry.launch.py

# 2. Проверка одометрии
ros2 topic hz /odom          # ~100Hz ✓
ros2 topic echo /odom --once # x,y,th ✓

# 3. RViz2
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix fourbox_odometry)/share/fourbox_odometry/rviz/odometry.rviz

# 4. Teleop тест
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -p speed:=0.3
```

## 📐 Кинематика Mecanum

```
Δticks_left  = (FL + RL) / 2
Δticks_right = (FR + RR) / 2
Δθ = theta_scale * (right-left) * 2πR / (PPR * wheel_base_x)
v_x = (left+right)/2 * 2πR / PPR
```

## 🎯 TODO

- [ ] **Научить робота сбрасывать тики энкодера при старте**  
  `serial_cmd: "ENC_RESET"` → `[0,0,0,0]` при запуске

- [ ] **Запуск публикации данных энкодера только в движении**  
  `if sum(abs(delta_ticks)) > threshold: publish()` → экономия трафика

## 🛠️ Структура пакета

```
fourbox_odometry/
├── launch/
│   └── fourbox_odometry.launch.py
├── config/
│   └── odometry_params.yaml
├── rviz/
│   └── odometry.rviz
├── fourbox_odometry/
│   ├── fourbox_odometry_node.py
│   └── __init__.py
├── package.xml
├── setup.py
└── README.md
```

## 🔗 Зависимости

```xml
rclpy
nav_msgs
geometry_msgs
tf2_ros
std_msgs
sensor_msgs
```

## 📄 Лицензия

MIT License - смотри [LICENSE](LICENSE)

***

**👨‍💻 Автор**: Robotics Developer  
**📧 Контакты**: Saint Petersburg, RU  
**🔗 ROS2 Jazzy** | **Mecanum Fourbox** | **100Гц Odometry** 🚀