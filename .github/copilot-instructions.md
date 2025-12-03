## Цель

Короткие практические подсказки для AI-агента, чтобы быстро стать продуктивным в этом репозитории ESP32 + ESP-IDF / PlatformIO.

## Ключевая архитектура (большая картина)

- Это прошивка для ESP32, организована как стандартный ESP-IDF CMake-проект: корневой `CMakeLists.txt` подключает `project.cmake` через `$IDF_PATH`.
- Альтернативный рабочий профиль — PlatformIO: в корне есть `platformio.ini` с `framework = espidf` и окружением `esp32dev`.
- Основная логика приложения в `src/` (см. `src/main.c`, `src/rotary_encoder.c`). В `lib/encoder/` — отдельная библиотека/компонент для работы с энкодерами (`encoder.c`, `encoder.h`).
- Есть два подхода к энкодеру в репозитории:
  - `src/rotary_encoder.c` — драйвер уровня компонента (ISR-based, state table, использует gpio interrupts).
  - `lib/encoder` — таймерный (esp_timer) вариант, опрашивает GPIO и публикует события в очередь.

Понимайте, что правки в логике энкодера затрагивают код, выполняемый в ISR или в таймерной обратной функции — соблюдайте ограничения IRAM/ISR.

## Как собирать и запускать (проверено в дереве проекта)

ESP-IDF (рекомендуемый для разработки и отладки):

```sh
cd /home/madrack/dev/Esp32_Espidf
# если ещё не экспортирован IDF, выполните экспорт окружения (зависит от установки ESP-IDF)
. $IDF_PATH/export.sh
idf.py set-target esp32
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

PlatformIO (альтернатива):

```sh
cd /home/madrack/dev/Esp32_Espidf
platformio run -e esp32dev
platformio run -t upload -e esp32dev -e esp32dev
platformio device monitor -b 115200
```

Serial monitor: `monitor_speed = 115200` — см. `platformio.ini`.

Если не уверены, спросите, какой workflow команда предпочитает (idf.py vs PlatformIO).

## Проектные соглашения и паттерны (конкретно для этого репозитория)

- Используются FreeRTOS-очереди для событий энкодера: драйвер создает/принимает события через Queue (см. `rotary_encoder_create_queue()` и `rotary_encoder_set_queue()` в `src/rotary_encoder.c`).
- Перед регистрацией энкодера требуется установить GPIO ISR service: `gpio_install_isr_service(0)` (см. `src/main.c`).
- Конфигурация энкодера задается в коде через макросы в `src/main.c` (например `ROT_ENC_A_GPIO`, `ENABLE_HALF_STEPS`). При изменениях лучше править эти макросы или делать их конфигируемыми через `sdkconfig`.
- При написании обработчиков прерываний соблюдайте IRAM и используйте безопасные-from-ISR API (в коде: `xQueueOverwriteFromISR`, `portYIELD_FROM_ISR`).
- Логирование: используется `ESP_LOGI/ESP_LOGE` и теги `TAG` внутри файлов (`"app"`, `"rotary_encoder"`, `"encoder"`). Сохраняйте существующий стиль логирования.

## Интеграционные точки и зависимости

- Зависимость от ESP-IDF (включая `freertos`, `driver/gpio`, `esp_timer`). См. `CMakeLists.txt` и `platformio.ini`.
- Локальные библиотеки/компоненты лежат в `lib/` (PlatformIO-подход) и/или могут быть вынесены в `components/` (ESP-IDF); текущая структура использует `lib/encoder` как библиотеку.

## Быстрые примеры правок (чего ожидает кодовая база)

- Добавить новую пару GPIO для энкодера: в `src/main.c` добавить/изменить макросы `ROT_ENC_A_GPIO`/`ROT_ENC_B_GPIO` и пересобрать.
- Переключиться между полным и полушаговым режимом: `rotary_encoder_enable_half_steps(&info, ENABLE_HALF_STEPS);`.
- Если правите ISR-логику, проверяйте, что функция помечена `IRAM_ATTR`, не вызывает блокирующие API и использует FromISR-версии FreeRTOS API.

## Файлы, которые нужно читать при внесении изменений (ключевые примеры)

- `CMakeLists.txt` — подключение ESP-IDF / корень проекта.
- `platformio.ini` — альтернативный профиль сборки и скорость монитора.
- `src/main.c` — точка входа приложения, создание задач, инициализация энкодера.
- `src/rotary_encoder.c` — ISR-based драйвер (state table, emits events via queue).
- `lib/encoder/encoder.c` и `lib/encoder/encoder.h` — timer-based encoder; пример использования `esp_timer` и mutex/queue.

## Что я не нашёл / вопросы для уточнения

- В репозитории нет существующего `.github/copilot-instructions.md` или AGENT-файлов — я создаю этот файл как стартовую точку.
- Уточните предпочтительный рабочий процесс: вы обычно используете `idf.py` (ESP-IDF) или PlatformIO для CI/локальной разработки?

## Завершение

Если нужно, могу:
- Перенести `lib/encoder` в `components/` (ESP-IDF style) и обновить `CMakeLists.txt` — сделаю аккуратно.
- Добавить примеры unit-интеграционных тестов для логики событий энкодера (PlatformIO test runner).

Пожалуйста, укажите, что добавить или уточнить в этом файле — внесу правки по обратной связи.
