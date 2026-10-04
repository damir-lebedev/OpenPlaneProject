<div dir="rtl">

# ‏CONFIG — ‏`Config` و`Channels` و`Controls`

> 🌐 هذه الصفحة ترجمة لـ[الأصل الروسي](../../../reference/config.md). إذا اختلفت الترجمة عن الأصل فالعبرة بالأصل. يعرض البرنامج الثابت رسائل وحدة التحكم (الكونسول) بالروسية، لذلك تُقتبس هنا كما هي.

[→ المرجع](README.md)

طبقة الضبط ثوابت `constexpr` فقط، بلا شيفرة. ولا ينبغي أن يحتوي منطق الأصناف على
أطراف أو مهلات أو عتبات «سحرية»: فكل ما قد يلزم تغييره لطائرة أو لوحة بعينها
مكانه هنا.

---

## ‏namespace `Config`

**الملف:** `include/config/Config.h` · **يعتمد على:** `<stdint.h>` ·
**تستخدمه:** معظم الطبقات.

### الأطراف (بحسب اللوحة)

تُختار كتلة الأطراف بالماكرو الذي يضبطه `[env:*]` في `platformio.ini`
(‏`-D BOARD_ESP32_S3` / `BOARD_ESP32_C3` / `BOARD_ESP32_CLASSIC` /
`BOARD_STM32H743`). ودون ماكرو — `#error`. وكتلة STM32 موصوفة
[أدناه](#stm32h743vit6-board_stm32h743).

| الثابت | النوع | الغرض | S3 | C3 | classic |
|---|---|---|---|---|---|
| `PIN_AILERON_LEFT` / `PIN_AILERON_RIGHT` | `uint8_t` | الجنيحان | 4 / 5 | 5 / 4 | 13 / 14 |
| `PIN_ELEVATOR` | `uint8_t` | الرافعة | 6 | 6 | 27 |
| `PIN_ESC` | `uint8_t` | منظِّم المحرك | 7 | 7 | 26 |
| `PIN_RUDDER` | `int8_t` | الدفة + العجلة؛ و`-1` — المخرج معطَّل | 18 | −1 | 25 |
| `PIN_IBUS` | `uint8_t` | RX لمستقبِل iBUS (‏UART1) | 17 | 8 | 16 |
| `PIN_I2C_SDA` / `PIN_I2C_SCL` | `uint8_t` | ناقل الحساسات (`Wire`) | 41 / 42 | 1 / 3 | 21 / 22 |
| `PIN_I2C2_SDA` / `PIN_I2C2_SCL` | `int8_t` | ناقل OLED (‏`Wire1`)؛ و`-1` — غير موجود | 1 / 2 | −1 | −1 |
| `PIN_SENSOR_SPI_SCK` / `MISO` / `MOSI` | `uint8_t` | ناقل SPI المشترك | 12 / 13 / 11 | 0 / 10 / 20 | 18 / 19 / 23 |
| `PIN_SPI_CS_IMU` | `uint8_t` | طرف CS لـ IMU عبر SPI | 14 | 21 | 32 |
| `PIN_SPI_CS_BARO` | `uint8_t` | طرف CS لمقياس الضغط عبر SPI | 21 | 2 | 5 |
| `PIN_GPS_RX` / `PIN_GPS_TX` | `int8_t` | UART الخاص بـ GPS؛ وTX بالقيمة `-1` — استقبال فقط | 39 / 40 | 9 / −1 | 4 / 17 |
| `UART_NUM_GPS` | `uint8_t` | رقم UART العتادي الخاص بـ GPS | 2 | 0 | 2 |
| `PIN_AUX1`، `PIN_AUX2` | `int8_t` | مخرجا سيرفو: إسقاط الحمولة، والفلابات؛ و`-1` — غير موجود | 15, 16 | −1, −1 | 33, 15 |
| `PIN_BUZZER` | `int8_t` | الصافرة عبر ترانزستور؛ و`-1` — غير موجودة | 38 | −1 | 2 |
| `PIN_AUX3` و`PIN_LIGHT` و`PIN_VBAT_ADC` و`PIN_CURRENT_ADC` و`PIN_TELEM_TX/RX` | `int8_t` | **S3 فقط:** محجوزة للوحة وحدة التحكم بالطيران ([FC_BOARD.md](../FC_BOARD.md)) | 47, 21, 8, 3, 9/10 | — | — |

يُسمّى ناقل SPI الخاص بالحساسات `PIN_SENSOR_SPI_*` وليس `PIN_SPI_*`: ففي نواة
STM32duino (وسائر أنوية Arduino) تكون `PIN_SPI_SCK/MISO/MOSI` ماكروات خاصة
بالنسخة، وكانت ستحل محل ثوابت `Config`.

<a id="stm32h743"></a>

#### ‏STM32H743VIT6 (‏`BOARD_STM32H743`)

لا توجد لوحة بعدُ: توزيع الأطراف **لم يُختبر على العتاد** (يُشغَّل البرنامج الثابت على الحاسوب، env ‏`native-stm32`). وقد اختيرت الأطراف من الحرة
على WeAct MiniSTM32H743VITx (لوحة env ‏`stm32h743` في PlatformIO) وطُوبقت مع
جداول `PeripheralPins` لنسخة STM32duino. والقيم ماكروات النسخة (`PA0`…)،
ولذلك يُضمَّن `<Arduino.h>` في بداية `Config.h` تحت
`#if defined(BOARD_STM32H743)`. ونوع جميع الأطراف هو `int16_t` (وترقيم
الأطراف التناظرية `0xC0 + N`). ولا توجد أرقام لـ UART — إذ تختار النواة الطرفية
بحسب الأطراف.

| الثابت | الطرف | الطرفية |
|---|---|---|
| `PIN_AILERON_LEFT` / `PIN_AILERON_RIGHT` / `PIN_ELEVATOR` / `PIN_ESC` | PA0 / PA1 / PA2 / PA3 | TIM2_CH1..CH4 |
| `PIN_RUDDER` | PD14 | TIM4_CH3 |
| `PIN_IBUS` / `PIN_IBUS_TX` | PE7 / PE8 | UART7 (‏TX — محجوز لـ iBUS-SENS) |
| `PIN_I2C_SDA` / `PIN_I2C_SCL` | PB11 / PB10 | I2C2 — الحساسات |
| `PIN_I2C2_SDA` / `PIN_I2C2_SCL` | PB9 / PB8 | I2C1 — الشاشة (وعلى WeAct — موصل الكاميرا) |
| `PIN_SENSOR_SPI_SCK` / `MISO` / `MOSI` | PB13 / PB14 / PB15 | SPI2 |
| `PIN_SPI_CS_IMU` / `PIN_SPI_CS_BARO` | PB12 / PD10 | GPIO |
| `PIN_GPS_RX` / `PIN_GPS_TX` | PD9 / PD8 | USART3 |
| `PIN_AUX1` / `PIN_AUX2` | PD15 / PE9 | TIM4_CH4 / TIM1_CH1 — الحمولة / الكاميرا |
| `PIN_BUZZER` | PE15 | GPIO — الصافرة |
| `PIN_VBAT_ADC` و`PIN_CURRENT_ADC` | PC0, PC1 | ADC1_INP10، ADC1_INP11 — محجوزان |
| `PIN_TELEM_RX` / `PIN_TELEM_TX` | PD0 / PD1 | UART4 — مودم راديو MAVLink (والأطراف نفسها هي FDCAN1) |

كونسول `Serial` هو LPUART1 (‏PA9 TX / PA10 RX)، وهو الإعداد الافتراضي للنسخة.

### ‏iBUS وفقدان الاتصال

| الثابت | القيمة | المعنى |
|---|---|---|
| `IBUS_CHANNELS` | 10 | عدد قنوات الإطار المستخدَمة |
| `IBUS_FRAME_LENGTH` | 32 | طول الإطار، بايت |
| `IBUS_HEADER_0` / `IBUS_HEADER_1` | `0x20` / `0x40` | ترويسة الإطار |
| `IBUS_BAUDRATE` | 115200 | سرعة UART |
| `RX_TIMEOUT_US` | 500 000 | لا إطار صحيحًا لمدة أطول من هذه — الاتصال مفقود |
| `RX_FAILSAFE_THROTTLE_US` | 950 | الخانق أدنى من هذه القيمة — يُبلغ المستقبِل عن failsafe جهاز التحكم |

### ‏GPS

| الثابت | القيمة | المعنى |
|---|---|---|
| `GPS_TIMEOUT_US` | 2 000 000 | NAV-PVT أقدم من هذه المدة — `UbloxM10_Gps::isAvailable() == false` |

### مدى PWM وأشواط الأسطح

| الثابت | القيمة | المعنى |
|---|---|---|
| `PWM_MIN` / `PWM_CENTER` / `PWM_MAX` | 1000 / 1500 / 2000 | نبضة RC القياسية، µs |
| `AILERON_MAX_US` و`ELEVATOR_MAX_US` و`RUDDER_MAX_US` | 500 / 500 / 300 | الانحراف عن المركز عند الشوط الكامل للعصا، µs. والدفة أصغر: فعجلة جهاز الهبوط على السيرفو نفسه |
| `THROTTLE_LIMIT_PCT` | 100 | سقف الخانق المتجه إلى ESC، بالنسبة المئوية، وهو نفسه للعصا وللطيار الآلي (`FlightController::capThrottle`). وفي اختبارات المنصة على حزمة 3S1P ضعيفة وُضعت القيمة 50؛ وتحسب الاختبارات المخرج المتوقع من هذه القيمة |

### الفلابات (فلابرون)

| الثابت | القيمة | المعنى |
|---|---|---|
| `FLAPS_SWITCH_ON_US` | 1750 | CH6 فوق هذه القيمة — الفلابات مُخرَجة (وليس 1500: فحتى أول إطار تكون القنوات = 1500) |
| `FLAPS_DEPLOYED_US` | 220 | انحراف كل جنيح إلى أسفل، µs (نحو 20° من ذراع MG90S) |
| `FLAPS_TRANSITION_MS` | 1000 | زمن الإخراج/السحب الكامل |

### اتجاه السيرفو

‏`AILERON_LEFT_REVERSED` و`AILERON_RIGHT_REVERSED` (القيمة `true` — سيرفوات الجنيحات مركَّبة بالانعكاس) و`ELEVATOR_REVERSED` (القيمة `true`)
و`RUDDER_REVERSED` — هي المكان الوحيد الذي يُحدَّد فيه العكس. ويحسب `ControlMixer`
بالإشارات الفيزيائية ويقلب الإشارة هنا فقط، ولذلك لا تختلف العصي
والطيار الآلي. ولا **يجوز** إجراء العكس في جهاز التحكم.

### تركيب الحساسات

| الثابت | القيمة | المعنى |
|---|---|---|
| `IMU_ROTATION_CW_DEG` | 90 | دوران محاور شريحة IMU حول المحور الرأسي (0/90/180/270)، أي إلى أين يشير المحور X للشريحة. ويُستخدم فقط ما دامت معايرة التركيب `o` غير موجودة في NVS |
| `MAG_ROTATION_CW_DEG` | 0 | الشيء نفسه للبوصلة (فلا معايرة تركيب للبوصلة) |

### ‏ARM

| الثابت | القيمة | المعنى |
|---|---|---|
| `ARM_SWITCH_ON_US` | 1750 | CH5 فوق هذه القيمة — مفتاح ARM في وضع التشغيل |
| `THROTTLE_LOW_US` | 1050 | الخانق أدنى من هذه القيمة — «الخانق في الأسفل»، ويمكن التفعيل |

### ‏Failsafe

| الثابت | القيمة | المعنى |
|---|---|---|
| `FAILSAFE_AILERON` / `ELEVATOR` / `RUDDER` | 1500 | الوضع المحايد للأسطح |
| `FAILSAFE_THROTTLE` | 1000 | المحرك مطفأ |
| `FAILSAFE_GLIDE_ROLL_DEG` | 0.0 | ميلان الانسياب عند فقدان الاتصال في الجو |
| `FAILSAFE_GLIDE_PITCH_DEG` | −3.0 | ميل الانسياب (أدنى قليلًا من الأفق) |
| `FAILSAFE_RTH` | `true` | مع توفر GPS ونقطة الانطلاق، فقدان الاتصال في الجو — عودة إلى نقطة الانطلاق بالمحرك لا انسياب |

### المفاتيح وأنبوب بيتو والطيار الآلي

أرقام جميع الأوضاع والوظائف في `Config.h` إلى جانب تعليقات مفصلة؛
وما تعنيه للطيار في [AUTOPILOT_GUIDE.md](../AUTOPILOT_GUIDE.md).

| المجموعة | الثوابت |
|---|---|
| المفاتيح | `SWITCH_ON_US` = 1750 (القناة فوق هذه القيمة — المفتاح مُشغَّل؛ وليس 1500 كي لا يُفعَّل شيء قبل أول إطار) |
| أنبوب بيتو | `PITOT_ZERO_SAMPLES` 50, `PITOT_FILTER_TAU_S` 0.1, `PITOT_SCALE` 1.0, `PITOT_NEGATIVE_FAULT_PA/MS` 30/2000, `PITOT_STALE_US` 200 000 |
| التثبيت | `MAX_BANK_DEG` 45 (المقبض 15…60), `STAB_MAX_PITCH_DEG` 25, `STAB_INTEGRATOR_ZONE_DEG` 10, `STAB_GAIN_MIN/MAX` 0.25/2 |
| ACRO | `ACRO_MAX_RATE_DPS` 180, `ACRO_RATE_GAIN_US_PER_DPS` 1.5 |
| الملاحة | `NAV_COURSE_GAIN`, `NAV_BANK_LIMIT_DEG` 40, `NAV_GPS_COURSE_MIN_SPEED_MS` 3, `NAV_ASSUMED_SPEED_MS` 15 |
| الارتفاع | `NAV_ALT_GAIN`, `NAV_MAX_CLIMB/SINK_MS` 3/3, `NAV_CLIMB_KP/KI_DEG`, `NAV_MAX_CLIMB/DIVE_PITCH_DEG` 15/−12 |
| الخانق والسرعة | `CRUISE_THROTTLE_PCT` 55 (30…85), `CRUISE_AIRSPEED_MS` 14 (10…22), `AIRSPEED_THROTTLE_KP/KI`, `THROTTLE_PER_CLIMB_PCT`, `AUTO_THROTTLE_MIN/MAX_PCT` |
| الانهيار | `STALL_SPEED_MS` 8, `STALL_MARGIN_MS` 2, `STALL_BANK_LIMIT_DEG` 20 |
| الدوائر ونقطة الانطلاق | `LOITER_RADIUS_M` 50 (25…150), `LOITER_CONVERGENCE`, `RTH_ALTITUDE_M` 40, `HOME_MIN_SATELLITES` 6, `HOME_MAX_HACC_M` 5 |
| السياج الجغرافي | `GEOFENCE_ALWAYS_ON` false, `FENCE_RADIUS_M` 500, `FENCE_ALTITUDE_M` 120 |
| الإطلاق باليد | `LAUNCH_ACCEL_G` 1.5, `LAUNCH_ACCEL_TIME_MS` 40, `LAUNCH_MOTOR_DELAY_MS` 300, `LAUNCH_THROTTLE_PCT`, `LAUNCH_CLIMB_PITCH_DEG` 15, `LAUNCH_CLIMB_MS` 6000, `LAUNCH_ALTITUDE_M` 30 |
| الهبوط | `LAND_GLIDE_PITCH_DEG` −4, `LAND_FLARE_ALTITUDE_M` 3, `LAND_FLARE_PITCH_DEG` 4 |
| التحليق الشراعي | `SOAR_*`: انسياب −3°، وتيار حراري > 0.5 m/s لمدة 1.5 s، ودائرة 25°، وخروج < −0.2 m/s لمدة 8 s، والمحرك دون 30 m حتى 100 m، والعودة إلى نقطة الانطلاق بعد 400 m |
| RESCUE | `RESCUE_PITCH_DEG` 8, `RESCUE_THROTTLE_PCT` 70 |
| الضبط التلقائي | `AUTOTRIM_RATE` 0.2, `AUTOTRIM_MAX_US` 120, `AUTOTRIM_MAX_ROLL_DEG` 15, `AUTOTRIM_MAX_RATE_DPS` 30, والحفظ على الأرض: `AUTOTRIM_SAVE_MAX_ALT_M` 3, `_CLIMB_MS` 0.5, `_SPEED_MS` 3 |
| التنسيق | `TURN_COORD_RUDDER_MIX` 0.3, `TURN_COORD_PITCH_US` 150, `TURN_COORD_IN_NAV_MODES` true |
| الوظائف | `AIRBRAKE_US` 250, `PAYLOAD_CLOSED/OPEN_US` 1000/2000, `CAMERA_TILT_MIN/MAX_DEG` −90/30, `CAMERA_US_PER_DEG`, `RATES_MIN/MAX_PCT` 30/100, `LOST_MODEL_BEEP_DELAY_MS` 10 000 |

### الحلقة وWi-Fi والتصحيح

| الثابت | القيمة | المعنى |
|---|---|---|
| `LOOP_PERIOD_MS` | 2 | دورة حلقة الطيران (500 Hz)؛ وهي أيضًا قيمة `dt` الاسمية لـ `PidController` |
| `WIFI_AP_SSID` / `WIFI_AP_PASSWORD` | `"OpenPlane-Debug"` / `"12345678"` | نقطة وصول لوحة المعلومات (كلمة المرور ضعيفة — أداة للمنصة) |
| `WEB_SERVER_PORT` | 80 | منفذ HTTP |
| `TELEM_BAUDRATE` | 57600 | سرعة مودم راديو MAVLink (الافتراضية في SiK) |
| `MAVLINK_SYSTEM_ID` / `MAVLINK_COMPONENT_ID` | 1 / 1 | عنوان الطائرة في MAVLink |
| `DEBUG_INTERVAL_MS` | 100 | كم مرة يفحص `DebugLogger` قنوات السجل |
| `DEBUG_CHANGE_DEADBAND_US` | 3 | هامش تحمّل ارتجاف RC/PWM في وضع «عند التغيّر» |

### الصندوق الأسود

بالتفصيل — [BLACKBOX.md](../BLACKBOX.md).

| الثابت | القيمة | المعنى |
|---|---|---|
| `BLACKBOX_RING_BYTES` / `_NO_PSRAM_BYTES` | 4 MB / 32 KB | طابور السجلات في PSRAM (ودون PSRAM — في الذاكرة الداخلية) |
| `BLACKBOX_RING_STM32_BYTES` | 384 KB | STM32H743: الطابور في الذاكرة العشوائية — 10 s من التسجيل المسبق وهامش لتأخر البطاقة |
| `BLACKBOX_SD_FILE` / `BLACKBOX_SD_MAX_BYTES` | `BLACKBOX.BIN` / 256 MB | STM32H743: الملف على بطاقة SD وسقف الجزء المستخدَم منه (يطول زمن المطابقة عند التشغيل مع اتساع المنطقة) |
| `BLACKBOX_PREROLL_MS` / `_POSTROLL_MS` | 10 000 / 10 000 | التسجيل قبل البدء (ARM + الخانق) وبعد DISARM |
| `BLACKBOX_LANDED_STOP_MS` | 30 000 | مُفعَّلة، والمحرك متوقف، والطائرة ساكنة طوال هذه المدة — إيقاف |
| `BLACKBOX_LANDED_GYRO_DPS` / `_ACCEL_G` / `_CLIMB_MS` / `_SPEED_MS` | 5 / 0.1 / 0.5 / 2 | ما الذي يُعدّ «ساكنًا» |
| `BLACKBOX_RESET_HOLD_MS` | 60 000 | بعد إعادة تشغيل معيبة — التسجيل لمدة لا تقل عن هذه |
| `BLACKBOX_MIN_FREE_BYTES` | 10 MB | مساحة ممسوحة جاهزة دائمًا؛ وتُمسح الرحلات القديمة كاملةً على الأرض |
| `BLACKBOX_ERASE_PAUSE_MS` | 100 | الفاصل بين عمليات المسح |
| `BLACKBOX_IMU_DIVIDER` | 1 | قراءة IMU مرة كل N دورة (1 — 500 Hz) |
| `BLACKBOX_VBAT_DIVIDER` / `_CURRENT_DIVIDER` | 6.6 / 1.667 | مقسِّما البطارية (56k/10k) وحساس التيار (10k/15k) على لوحة وحدة التحكم بالطيران |

---

## ‏namespace `Channels`

**الملف:** `include/config/Channels.h` · **يعتمد على:** `<stdint.h>`

المكان الوحيد الذي يُربط فيه الرقم الفيزيائي للقناة بالغرض منها. والقيم
**فهارس** (تبدأ من 0) في `RcChannelState`.

| الثابت | الفهرس | القناة | عنصر FS-i6 | الغرض |
|---|---|---|---|---|
| `AILERON` | 0 | CH1 | العصا اليمنى ←→ | الدوران الجانبي |
| `ELEVATOR` | 1 | CH2 | العصا اليمنى ↑↓ | الميل (2000 = بعيدًا عنك = الأنف إلى أسفل) |
| `THROTTLE` | 2 | CH3 | العصا اليسرى ↑↓ | الخانق |
| `RUDDER` | 3 | CH4 | العصا اليسرى ←→ | الدفة + العجلة |
| `ARM` | 4 | CH5 | SwA | مفتاح ARM (لا يمكن إعادة إسناده) |
| `SWB` | 5 | CH6 | SwB | بحسب جدول `Controls.h` (افتراضيًّا الفلابات) |
| `SWC` | 6 | CH7 | SwC (3 أوضاع) | افتراضيًّا الوضع MANUAL / STABILIZE / AUTO_TAKEOFF |
| `SWD` | 7 | CH8 | SwD | افتراضيًّا RTH |
| `VRA` | 8 | CH9 | VrA | افتراضيًّا `STAB_GAIN` |
| `VRB` | 9 | CH10 | VrB | افتراضيًّا `CRUISE_SPEED` |
| `COUNT` | 10 | | | عدد القنوات |

---

## ‏namespace `Controls`

**الملف:** `include/config/Controls.h` · **يعتمد على:** `ControlBinding.h` و`Channels`

‏`constexpr Binding BINDINGS[]` — ما يفعله كل مفتاح ومقبض، **سطر واحد لكل
قناة** (`Bind::modes/mode/feature/knob`، انظر
[autopilot.md](autopilot.md#binding-وbind-وbindingcheck)). وإلى جانبه أفكار
جاهزة معلَّقة بالتعليقات. وثلاثة `static_assert` تلتقط أخطاء الجدول عند البناء:
وجود عصا أو ARM في الجدول، وقناة خارج المدى، وقناة مكررة، وأكثر من مفتاح
واحد لاختيار الوضع.

</div>
