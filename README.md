# Симуляция Robomarvel

Симулятор для курса «Интеллектуальная робототехника с ROS 2». Заменяет **только железо** —
драйвер платы, драйвер лидара и источник видео. Всё, что выше (ICP-одометрия, SLAM, Nav2),
запускается настоящими нодами с конфигами робота, поэтому код из симуляции переносится
на железо (по идее)

```
        СИМУЛЯЦИЯ                       │              НАСТОЯЩЕЕ
  sim_hwnode    → /hardware/*, /cmd_vel │  icp_odometry → /icp/odom, TF odom→base_link
  sim_lidar     → /scan (кадр lidar)    │  rtabmap      → /map,      TF map→odom
  sim_detection → /detections           │  Nav2         → /plan, /cmd_vel
                                        │  goal_proxy   → /goal/*
                                        │  robot_state_publisher (robot.urdf с робота)
```

---

## Быстрый старт

```bash
docker build -t robomarvel-sim .
docker run -it --name robomarvel-sim \
  -v "$(pwd)":/home/student/ros2_ws \
  -p 127.0.0.1:8765:8765 \
  robomarvel-sim
```

Внутри контейнера:

```bash
python3 selfcheck.py     # синтаксис, YAML, URDF, математика - 2 секунды, без ROS
cb && cs                 # colcon build --symlink-install + source
ros2 launch robomarvel_sim sim.launch.py odom:=perfect slam:=off nav:=false detection:=false
```

---

## Ноды

| Нода | Заменяет на роботе | Публикует |
|---|---|---|
| `sim_hwnode` | `hwnode` (STM32 по UART) | `/hardware/status`, `/hardware/odom`, `/hardware/imu`, `/hardware/battery`, `/sim/ground_truth` |
| `sim_lidar` | `ld19_node` (лидар) | `/scan`, 450 лучей, 10 Гц, кадр `lidar` |
| `sim_detection` | `detection` (частично) | `/detections`, `/camera/camera_info`, `/camera/image_annotated_compressed` |
| `goal_proxy` | копия ноды с робота | `/goal/status`, `/goal/feedback`, сервис `/goal/cancel` |

Остальное запускается как есть: `robot_state_publisher`, `icp_odometry` (`rtabmap_odom`),
`rtabmap` (`rtabmap_slam`), Nav2 (`controller_server`, `planner_server`, `behavior_server`,
`bt_navigator`, `lifecycle_manager`), `foxglove_bridge`.

`sim_hwnode` **не публикует** `odom → base_link` — эту трансформацию считает настоящий
`icp_odometry` по нашим сканам. Истинная поза уходит в отдельное дерево `sim_world → sim_base`,
не связанное с деревом робота, чтобы ни одна нода не могла на неё опереться. Разница между
`/icp/odom` и `/sim/ground_truth` — это и есть дрейф одометрии.

## Launch

Один файл на всё, собирается по частмя:

```
sim.launch.py  world:=  odom:=icp|perfect  slam:=mapping|localization|off  nav:=  detection:=  db:=
```

| Аргумент | По умолчанию | Смысл |
|---|---|---|
| `odom` | `icp` | `perfect` — TF из истинной позы, только для отладки |
| `slam` | `mapping` | `localization` — по готовой базе, `off` — статический `map→odom` |
| `nav` | `true` | стек Nav2 + `goal_proxy` |
| `detection` | `true` | виртуальная детекция объектов |
| `db` | `~/ros2_ws/maps/rtabmap.db` | база rtabmap |
| `world` | `config/worlds/classroom.yaml` | планировка |

⚠️ `slam:=mapping` **стирает базу при старте**. Готовую карту храните копией и указывайте `db:=`.

---

## Валидация по частям

Каждая включает предыдущую. Между запусками — Ctrl+C **один раз** и `ros2 node list` должен
быть пуст: два launch одновременно дают дубли

Управление во втором терминале (`cs` обязателен):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

если вдруг не работает, то:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p repeat_rate:=10.0
```

| № | Команда (доп. к `sim.launch.py`) | Что проверяем |
|---|---|---|
| 1 | `odom:=perfect slam:=off nav:=false detection:=false` | `/scan` ~10 Гц, `/hardware/status` ~50 Гц, `/hardware/battery` ~8.3, TF `map→odom→base_link→lidar` |
| 2 | `odom:=icp slam:=off nav:=false detection:=false` | `/icp/odom` ~10 Гц; в логе `Odom: ratio=0.7-0.8` |
| 3 | `slam:=mapping nav:=false detection:=false` | `/map` нарастает при объезде; база в `maps/` |
| 4 | `slam:=localization db:=... nav:=false detection:=false` | `/map` появляется сразу целиком |
| 5 | `slam:=localization db:=... nav:=true detection:=false` | цель в `/goal` → робот едет |
| 6 | `slam:=localization db:=... nav:=true detection:=true` | `/detections` с классами и дальностями |

**Ступень 2 — основная.** Если настоящий ICP сходится на сканах в симуляторе, значит модель
лидара достаточно правдоподобна.

**Ступень 5.** Цель можно послать из терминала либо через кнопку в Foxglove/Lichtblick (в лихтблике надо поменять топик на /goal в настройках 3D Panel в Publish)
(По умолчанию там `/move_base_simple/goal`)

```bash
ros2 topic pub --once -w 1 /goal geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: -0.5, z: 0.0}, orientation: {w: 1.0}}}"
ros2 topic echo /goal/status     # GOAL ACCEPTED ... SUCCESS
```

Карта сохраняется сама при Ctrl+C — отдельной команды нет, база и есть карта.
Для `.pgm` + `.yaml` (только для просмотра, локализация ими не пользуется):
`ros2 run nav2_map_server map_saver_cli -f maps/classroom` 

---

## Foxglove / Lichtblick

**Веб-версия по HTTPS не подключится к `ws://localhost:8765`.** Начиная с Chrome 147
действует Local Network Access (публичная страница не может открыть сокет на loopback).
Сейчас еще работает Lichtblick в Яндекс Браузере, но решением скорее всего является только Foxglove в контейнере

Настройка панелей:

- 3D: display frame `map`; топики `/scan` (point size 3–4), `/global_costmap/costmap`, `/plan`.
  **`/map` и костмап одновременно не включайте** — map перекрывает карту стоимости.
- Transforms: `sim_world` и `sim_base` выключить, иначе панель ругается на отсутствие связи
  с `map` (они в отдельном дереве намеренно).
- Image: `/camera/image_annotated_compressed`.

---

## Мир

`config/worlds/classroom.yaml` — класс 8×6 м: `walls` (отрезки), `boxes` (прямоугольники),
`circles`, `objects` (они же препятствия для лидара), `start`. Геометрия подобрана так,
чтобы ICP было за что цепляться: углы, две перегородки, мебель с плоскими гранями.

---

## Известные ограничения

- **Камера.** Видео пока нет: ожидается отдельная нода, стримящая вебку из браузера.
- **Звук.** Не реализован. Docker Desktop не пробрасывает `/dev/snd`, нужен проброс
  через браузер видимо тоже.
- **Релокализация rtabmap.** В режиме `localization` стартует от последней позы предыдущей
  сессии и держит её, пока не узнает местность. Нужен короткий проезд либо подсказка
  через `/initialpose`.

