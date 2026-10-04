# CONFIG — `Config`, `Channels`, `Controls`

> 🌐 यह पृष्ठ [रूसी मूल पाठ](../../../reference/config.md) का अनुवाद है। अनुवाद और मूल पाठ में अंतर हो, तो मूल पाठ ही मान्य है। फ़र्मवेयर कंसोल संदेश रूसी में दिखाता है, इसलिए उन्हें जैसा है वैसा ही उद्धृत किया गया है।

[← संदर्भ](README.md)

कॉन्फ़िगरेशन परत केवल `constexpr` स्थिरांक है, कोड के बिना। क्लासों के तर्क में
“जादुई” पिन, टाइमआउट और सीमाएँ नहीं होनी चाहिए: जो कुछ किसी ख़ास विमान या बोर्ड के
लिए बदलना पड़ सकता है, वह यहीं रहता है।

---

## namespace `Config`

**फ़ाइल:** `include/config/Config.h` · **निर्भर है:** `<stdint.h>` ·
**इस्तेमाल करती हैं:** लगभग सभी परतें।

### पिन (बोर्ड पर निर्भर)

पिनों का ब्लॉक उस मैक्रो से चुना जाता है जो `platformio.ini` में `[env:*]` तय करता है
(`-D BOARD_ESP32_S3` / `BOARD_ESP32_C3` / `BOARD_ESP32_CLASSIC` /
`BOARD_STM32H743`)। मैक्रो के बिना — `#error`। STM32 का ब्लॉक
[नीचे](#stm32h743vit6-board_stm32h743) वर्णित है।

| स्थिरांक | प्रकार | उपयोग | S3 | C3 | classic |
|---|---|---|---|---|---|
| `PIN_AILERON_LEFT` / `PIN_AILERON_RIGHT` | `uint8_t` | एलेरॉन | 4 / 5 | 5 / 4 | 13 / 14 |
| `PIN_ELEVATOR` | `uint8_t` | एलिवेटर | 6 | 6 | 27 |
| `PIN_ESC` | `uint8_t` | मोटर का रेगुलेटर | 7 | 7 | 26 |
| `PIN_RUDDER` | `int8_t` | रडर + पहिया; `-1` — आउटपुट बंद | 18 | −1 | 25 |
| `PIN_IBUS` | `uint8_t` | iBUS रिसीवर का RX (UART1) | 17 | 8 | 16 |
| `PIN_I2C_SDA` / `PIN_I2C_SCL` | `uint8_t` | सेंसरों की बस (`Wire`) | 41 / 42 | 1 / 3 | 21 / 22 |
| `PIN_I2C2_SDA` / `PIN_I2C2_SCL` | `int8_t` | OLED की बस (`Wire1`); `-1` — नहीं | 1 / 2 | −1 | −1 |
| `PIN_SENSOR_SPI_SCK` / `MISO` / `MOSI` | `uint8_t` | साझा SPI बस | 12 / 13 / 11 | 0 / 10 / 20 | 18 / 19 / 23 |
| `PIN_SPI_CS_IMU` | `uint8_t` | SPI पर IMU का CS | 14 | 21 | 32 |
| `PIN_SPI_CS_BARO` | `uint8_t` | SPI पर बैरोमीटर का CS | 21 | 2 | 5 |
| `PIN_GPS_RX` / `PIN_GPS_TX` | `int8_t` | GPS का UART; TX `-1` — केवल प्राप्ति | 39 / 40 | 9 / −1 | 4 / 17 |
| `UART_NUM_GPS` | `uint8_t` | GPS के लिए हार्डवेयर UART का नंबर | 2 | 0 | 2 |
| `PIN_AUX1`, `PIN_AUX2` | `int8_t` | सर्वो आउटपुट: पेलोड गिराना, फ़्लैप; `-1` — नहीं | 15, 16 | −1, −1 | 33, 15 |
| `PIN_BUZZER` | `int8_t` | ट्रांज़िस्टर के ज़रिए बजर; `-1` — नहीं | 38 | −1 | 2 |
| `PIN_AUX3`, `PIN_LIGHT`, `PIN_VBAT_ADC`, `PIN_CURRENT_ADC`, `PIN_TELEM_TX/RX` | `int8_t` | **केवल S3:** फ़्लाइट-कंट्रोलर बोर्ड के लिए आरक्षित ([FC_BOARD.md](../FC_BOARD.md)) | 47, 21, 8, 3, 9/10 | — | — |

सेंसरों की SPI बस का नाम `PIN_SENSOR_SPI_*` है, `PIN_SPI_*` नहीं: STM32duino कोर (और
दूसरे Arduino कोर) में `PIN_SPI_SCK/MISO/MOSI` वेरिएंट के मैक्रो हैं, वे `Config` के
स्थिरांकों की जगह ले लेते।

<a id="stm32h743"></a>

#### STM32H743VIT6 (`BOARD_STM32H743`)

अभी बोर्ड नहीं है: पिन-व्यवस्था **हार्डवेयर पर जाँची नहीं गई** (फ़र्मवेयर PC पर चलाया जाता है, env `native-stm32`)। पिन WeAct MiniSTM32H743VITx
(PlatformIO का env `stm32h743` बोर्ड) पर ख़ाली पिनों में से चुने गए और
STM32duino वेरिएंट की `PeripheralPins` तालिकाओं से मिलाए गए हैं। मान वेरिएंट के
मैक्रो (`PA0`…) हैं, इसलिए `Config.h` की शुरुआत में `#if defined(BOARD_STM32H743)`
के तहत `<Arduino.h>` शामिल किया जाता है। सभी पिनों का प्रकार `int16_t` है (एनालॉग
पिनों का नंबर `0xC0 + N`)। UART के नंबर नहीं हैं — पेरिफ़ेरल कोर पिनों से चुनता है।

| स्थिरांक | पिन | पेरिफ़ेरल |
|---|---|---|
| `PIN_AILERON_LEFT` / `PIN_AILERON_RIGHT` / `PIN_ELEVATOR` / `PIN_ESC` | PA0 / PA1 / PA2 / PA3 | TIM2_CH1..CH4 |
| `PIN_RUDDER` | PD14 | TIM4_CH3 |
| `PIN_IBUS` / `PIN_IBUS_TX` | PE7 / PE8 | UART7 (TX — iBUS-SENS के लिए आरक्षित) |
| `PIN_I2C_SDA` / `PIN_I2C_SCL` | PB11 / PB10 | I2C2 — सेंसर |
| `PIN_I2C2_SDA` / `PIN_I2C2_SCL` | PB9 / PB8 | I2C1 — स्क्रीन (WeAct पर — कैमरे का कनेक्टर) |
| `PIN_SENSOR_SPI_SCK` / `MISO` / `MOSI` | PB13 / PB14 / PB15 | SPI2 |
| `PIN_SPI_CS_IMU` / `PIN_SPI_CS_BARO` | PB12 / PD10 | GPIO |
| `PIN_GPS_RX` / `PIN_GPS_TX` | PD9 / PD8 | USART3 |
| `PIN_AUX1` / `PIN_AUX2` | PD15 / PE9 | TIM4_CH4 / TIM1_CH1 — पेलोड / कैमरा |
| `PIN_BUZZER` | PE15 | GPIO — बजर |
| `PIN_VBAT_ADC`, `PIN_CURRENT_ADC` | PC0, PC1 | ADC1_INP10, ADC1_INP11 — आरक्षित |
| `PIN_TELEM_RX` / `PIN_TELEM_TX` | PD0 / PD1 | UART4 — MAVLink का रेडियो मॉडेम (यही पिन FDCAN1 भी हैं) |

कंसोल `Serial` — LPUART1 (PA9 TX / PA10 RX), वेरिएंट का डिफ़ॉल्ट।

### iBUS और संपर्क टूटना

| स्थिरांक | मान | अर्थ |
|---|---|---|
| `IBUS_CHANNELS` | 10 | फ़्रेम के कितने चैनल इस्तेमाल होते हैं |
| `IBUS_FRAME_LENGTH` | 32 | फ़्रेम की लंबाई, बाइट |
| `IBUS_HEADER_0` / `IBUS_HEADER_1` | `0x20` / `0x40` | फ़्रेम का हेडर |
| `IBUS_BAUDRATE` | 115200 | UART की गति |
| `RX_TIMEOUT_US` | 500 000 | इससे ज़्यादा देर तक कोई सही फ़्रेम नहीं — संपर्क खो गया |
| `RX_FAILSAFE_THROTTLE_US` | 950 | थ्रॉटल इससे नीचे — रिसीवर ट्रांसमीटर का failsafe बता रहा है |

### GPS

| स्थिरांक | मान | अर्थ |
|---|---|---|
| `GPS_TIMEOUT_US` | 2 000 000 | इससे पुराना NAV-PVT — `UbloxM10_Gps::isAvailable() == false` |

### PWM की सीमा और कंट्रोल सरफ़ेस की यात्रा

| स्थिरांक | मान | अर्थ |
|---|---|---|
| `PWM_MIN` / `PWM_CENTER` / `PWM_MAX` | 1000 / 1500 / 2000 | मानक RC पल्स, µs |
| `AILERON_MAX_US`, `ELEVATOR_MAX_US`, `RUDDER_MAX_US` | 500 / 500 / 300 | स्टिक की पूरी यात्रा पर केंद्र से विक्षेपण, µs। रडर कम है: उसी सर्वो पर लैंडिंग गियर का पहिया है |
| `THROTTLE_LIMIT_PCT` | 100 | ESC को जाने वाले थ्रॉटल की छत, %, स्टिक और ऑटोपायलट के लिए एक जैसी (`FlightController::capThrottle`)। कमज़ोर 3S1P बैटरी पर टेस्ट बेंच की जाँच के लिए 50 रखा गया था; टेस्ट अपेक्षित आउटपुट इसी मान से गिनते हैं |

### फ़्लैप (फ़्लैपरॉन)

| स्थिरांक | मान | अर्थ |
|---|---|---|
| `FLAPS_SWITCH_ON_US` | 1750 | CH6 इससे ऊपर — फ़्लैप निकले हुए (1500 नहीं: पहले फ़्रेम तक चैनल = 1500) |
| `FLAPS_DEPLOYED_US` | 220 | हर एलेरॉन का नीचे की ओर विक्षेपण, µs (MG90S के हॉर्न के ~20°) |
| `FLAPS_TRANSITION_MS` | 1000 | पूरा निकलने/समेटने का समय |

### सर्वो की दिशा

`AILERON_LEFT_REVERSED`, `AILERON_RIGHT_REVERSED` (`true` — एलेरॉन के सर्वो आईने जैसे लगे हैं), `ELEVATOR_REVERSED` (`true`),
`RUDDER_REVERSED` — रिवर्स तय करने की इकलौती जगह। `ControlMixer` भौतिक चिह्नों में
गिनता है और चिह्न केवल यहीं बदलता है, इसलिए स्टिक और ऑटोपायलट अलग-अलग
दिशा में नहीं जा सकते। ट्रांसमीटर पर रिवर्स **नहीं** करना चाहिए।

### सेंसरों की माउंटिंग

| स्थिरांक | मान | अर्थ |
|---|---|---|
| `IMU_ROTATION_CW_DEG` | 90 | IMU चिप की अक्षों का ऊर्ध्वाधर के चारों ओर घुमाव (0/90/180/270), यानी चिप की X अक्ष किधर देखती है। केवल तब इस्तेमाल होता है जब NVS में माउंटिंग का कैलिब्रेशन `o` नहीं है |
| `MAG_ROTATION_CW_DEG` | 0 | कम्पास के लिए वही (कम्पास में माउंटिंग का कैलिब्रेशन नहीं है) |

### ARM

| स्थिरांक | मान | अर्थ |
|---|---|---|
| `ARM_SWITCH_ON_US` | 1750 | CH5 इससे ऊपर — ARM स्विच चालू है |
| `THROTTLE_LOW_US` | 1050 | थ्रॉटल इससे नीचे — “थ्रॉटल नीचे”, ARM किया जा सकता है |

### Failsafe

| स्थिरांक | मान | अर्थ |
|---|---|---|
| `FAILSAFE_AILERON` / `ELEVATOR` / `RUDDER` | 1500 | कंट्रोल सरफ़ेस का न्यूट्रल |
| `FAILSAFE_THROTTLE` | 1000 | मोटर बंद |
| `FAILSAFE_GLIDE_ROLL_DEG` | 0.0 | हवा में संपर्क टूटने पर ग्लाइड का रोल |
| `FAILSAFE_GLIDE_PITCH_DEG` | −3.0 | ग्लाइड की पिच (क्षितिज से ज़रा नीचे) |
| `FAILSAFE_RTH` | `true` | GPS और होम होने पर हवा में संपर्क टूटना — मोटर के साथ घर वापसी, ग्लाइड नहीं |

### स्विच, पिटो ट्यूब, ऑटोपायलट

सभी मोड और फ़ंक्शन की संख्याएँ `Config.h` में विस्तृत टिप्पणियों के साथ हैं;
पायलट के लिए उनका क्या अर्थ है — [AUTOPILOT_GUIDE.md](../AUTOPILOT_GUIDE.md) में।

| समूह | स्थिरांक |
|---|---|
| स्विच | `SWITCH_ON_US` = 1750 (चैनल इससे ऊपर — स्विच चालू; 1500 नहीं, ताकि पहले फ़्रेम से पहले कुछ चालू न हो) |
| पिटो ट्यूब | `PITOT_ZERO_SAMPLES` 50, `PITOT_FILTER_TAU_S` 0.1, `PITOT_SCALE` 1.0, `PITOT_NEGATIVE_FAULT_PA/MS` 30/2000, `PITOT_STALE_US` 200 000 |
| स्थिरीकरण | `MAX_BANK_DEG` 45 (नॉब 15…60), `STAB_MAX_PITCH_DEG` 25, `STAB_INTEGRATOR_ZONE_DEG` 10, `STAB_GAIN_MIN/MAX` 0.25/2 |
| ACRO | `ACRO_MAX_RATE_DPS` 180, `ACRO_RATE_GAIN_US_PER_DPS` 1.5 |
| नेविगेशन | `NAV_COURSE_GAIN`, `NAV_BANK_LIMIT_DEG` 40, `NAV_GPS_COURSE_MIN_SPEED_MS` 3, `NAV_ASSUMED_SPEED_MS` 15 |
| ऊँचाई | `NAV_ALT_GAIN`, `NAV_MAX_CLIMB/SINK_MS` 3/3, `NAV_CLIMB_KP/KI_DEG`, `NAV_MAX_CLIMB/DIVE_PITCH_DEG` 15/−12 |
| थ्रॉटल और गति | `CRUISE_THROTTLE_PCT` 55 (30…85), `CRUISE_AIRSPEED_MS` 14 (10…22), `AIRSPEED_THROTTLE_KP/KI`, `THROTTLE_PER_CLIMB_PCT`, `AUTO_THROTTLE_MIN/MAX_PCT` |
| स्टॉल | `STALL_SPEED_MS` 8, `STALL_MARGIN_MS` 2, `STALL_BANK_LIMIT_DEG` 20 |
| घेरे और होम | `LOITER_RADIUS_M` 50 (25…150), `LOITER_CONVERGENCE`, `RTH_ALTITUDE_M` 40, `HOME_MIN_SATELLITES` 6, `HOME_MAX_HACC_M` 5 |
| जियोफ़ेंस | `GEOFENCE_ALWAYS_ON` false, `FENCE_RADIUS_M` 500, `FENCE_ALTITUDE_M` 120 |
| हाथ से लॉन्च | `LAUNCH_ACCEL_G` 1.5, `LAUNCH_ACCEL_TIME_MS` 40, `LAUNCH_MOTOR_DELAY_MS` 300, `LAUNCH_THROTTLE_PCT`, `LAUNCH_CLIMB_PITCH_DEG` 15, `LAUNCH_CLIMB_MS` 6000, `LAUNCH_ALTITUDE_M` 30 |
| लैंडिंग | `LAND_GLIDE_PITCH_DEG` −4, `LAND_FLARE_ALTITUDE_M` 3, `LAND_FLARE_PITCH_DEG` 4 |
| सोअरिंग | `SOAR_*`: ग्लाइड −3°, थर्मल > 0.5 m/s 1.5 s तक, 25° का घेरा, निकास < −0.2 m/s 8 s तक, 30 m से नीचे मोटर 100 m तक, 400 m से आगे घर वापसी |
| RESCUE | `RESCUE_PITCH_DEG` 8, `RESCUE_THROTTLE_PCT` 70 |
| ऑटो-ट्रिम | `AUTOTRIM_RATE` 0.2, `AUTOTRIM_MAX_US` 120, `AUTOTRIM_MAX_ROLL_DEG` 15, `AUTOTRIM_MAX_RATE_DPS` 30, ज़मीन पर सहेजना: `AUTOTRIM_SAVE_MAX_ALT_M` 3, `_CLIMB_MS` 0.5, `_SPEED_MS` 3 |
| समन्वय | `TURN_COORD_RUDDER_MIX` 0.3, `TURN_COORD_PITCH_US` 150, `TURN_COORD_IN_NAV_MODES` true |
| फ़ंक्शन | `AIRBRAKE_US` 250, `PAYLOAD_CLOSED/OPEN_US` 1000/2000, `CAMERA_TILT_MIN/MAX_DEG` −90/30, `CAMERA_US_PER_DEG`, `RATES_MIN/MAX_PCT` 30/100, `LOST_MODEL_BEEP_DELAY_MS` 10 000 |

### चक्र, Wi-Fi, डीबगिंग

| स्थिरांक | मान | अर्थ |
|---|---|---|
| `LOOP_PERIOD_MS` | 2 | उड़ान-चक्र की अवधि (500 Hz); साथ ही `PidController` का नाममात्र `dt` |
| `WIFI_AP_SSID` / `WIFI_AP_PASSWORD` | `"OpenPlane-Debug"` / `"12345678"` | डैशबोर्ड का एक्सेस पॉइंट (पासवर्ड कमज़ोर है — टेस्ट बेंच का औज़ार) |
| `WEB_SERVER_PORT` | 80 | HTTP पोर्ट |
| `TELEM_BAUDRATE` | 57600 | MAVLink रेडियो मॉडेम की गति (SiK का डिफ़ॉल्ट) |
| `MAVLINK_SYSTEM_ID` / `MAVLINK_COMPONENT_ID` | 1 / 1 | MAVLink में विमान का पता |
| `DEBUG_INTERVAL_MS` | 100 | `DebugLogger` लॉग के चैनल कितनी बार जाँचता है |
| `DEBUG_CHANGE_DEADBAND_US` | 3 | “बदलाव पर” मोड में RC/PWM की थरथराहट की सहनशीलता |

### ब्लैक बॉक्स

विस्तार से — [BLACKBOX.md](../BLACKBOX.md)।

| स्थिरांक | मान | अर्थ |
|---|---|---|
| `BLACKBOX_RING_BYTES` / `_NO_PSRAM_BYTES` | 4 MB / 32 KB | PSRAM में रिकॉर्डों की कतार (PSRAM के बिना — भीतरी मेमोरी में) |
| `BLACKBOX_RING_STM32_BYTES` | 384 KB | STM32H743: RAM में कतार — 10 s की पूर्व-रिकॉर्डिंग और कार्ड की देरी के लिए गुंजाइश |
| `BLACKBOX_SD_FILE` / `BLACKBOX_SD_MAX_BYTES` | `BLACKBOX.BIN` / 256 MB | STM32H743: SD कार्ड पर फ़ाइल और उसके उपयोग किए जाने वाले हिस्से की सीमा (चालू करते समय मिलान का समय क्षेत्र के साथ बढ़ता है) |
| `BLACKBOX_PREROLL_MS` / `_POSTROLL_MS` | 10 000 / 10 000 | शुरुआत (ARM + थ्रॉटल) से पहले और DISARM के बाद की रिकॉर्डिंग |
| `BLACKBOX_LANDED_STOP_MS` | 30 000 | ARM है, मोटर रुकी है, विमान इतनी देर बिना हिले — रोकें |
| `BLACKBOX_LANDED_GYRO_DPS` / `_ACCEL_G` / `_CLIMB_MS` / `_SPEED_MS` | 5 / 0.1 / 0.5 / 2 | “बिना हिले” किसे माना जाता है |
| `BLACKBOX_RESET_HOLD_MS` | 60 000 | विफल रीबूट के बाद — रिकॉर्डिंग इससे कम नहीं |
| `BLACKBOX_MIN_FREE_BYTES` | 10 MB | तैयार रखी गई मिटाई हुई जगह; पुरानी उड़ानें ज़मीन पर पूरी की पूरी मिटाई जाती हैं |
| `BLACKBOX_ERASE_PAUSE_MS` | 100 | मिटाने के बीच का विराम |
| `BLACKBOX_IMU_DIVIDER` | 1 | IMU हर N-वें चक्र में (1 — 500 Hz) |
| `BLACKBOX_VBAT_DIVIDER` / `_CURRENT_DIVIDER` | 6.6 / 1.667 | फ़्लाइट-कंट्रोलर बोर्ड पर बैटरी (56k/10k) और करंट सेंसर (10k/15k) के डिवाइडर |

---

## namespace `Channels`

**फ़ाइल:** `include/config/Channels.h` · **निर्भर है:** `<stdint.h>`

इकलौती जगह जहाँ चैनल का भौतिक नंबर उपयोग से जोड़ा जाता है। मान `RcChannelState` में
**इंडेक्स** (0 से शुरू) हैं।

| स्थिरांक | इंडेक्स | चैनल | FS-i6 का हिस्सा | उपयोग |
|---|---|---|---|---|
| `AILERON` | 0 | CH1 | दायाँ स्टिक ←→ | रोल |
| `ELEVATOR` | 1 | CH2 | दायाँ स्टिक ↑↓ | पिच (2000 = अपने से दूर = नाक नीचे) |
| `THROTTLE` | 2 | CH3 | बायाँ स्टिक ↑↓ | थ्रॉटल |
| `RUDDER` | 3 | CH4 | बायाँ स्टिक ←→ | रडर + पहिया |
| `ARM` | 4 | CH5 | SwA | ARM स्विच (बदला नहीं जा सकता) |
| `SWB` | 5 | CH6 | SwB | `Controls.h` की तालिका के अनुसार (डिफ़ॉल्ट रूप से फ़्लैप) |
| `SWC` | 6 | CH7 | SwC (3 स्थिति) | डिफ़ॉल्ट रूप से मोड MANUAL / STABILIZE / AUTO_TAKEOFF |
| `SWD` | 7 | CH8 | SwD | डिफ़ॉल्ट रूप से RTH |
| `VRA` | 8 | CH9 | VrA | डिफ़ॉल्ट रूप से `STAB_GAIN` |
| `VRB` | 9 | CH10 | VrB | डिफ़ॉल्ट रूप से `CRUISE_SPEED` |
| `COUNT` | 10 | | | चैनलों की संख्या |

---

## namespace `Controls`

**फ़ाइल:** `include/config/Controls.h` · **निर्भर है:** `ControlBinding.h`, `Channels`

`constexpr Binding BINDINGS[]` — हर स्विच और नॉब क्या करता है, **हर चैनल की एक
पंक्ति** (`Bind::modes/mode/feature/knob`, देखें
[autopilot.md](autopilot.md#binding-bind-bindingcheck))। पास में टिप्पणी में रखे गए
तैयार विचार हैं। तीन `static_assert` बिल्ड के समय तालिका की ग़लतियाँ पकड़ते हैं:
तालिका में स्टिक या ARM, सीमा से बाहर का चैनल, दोहराया हुआ चैनल, मोड चुनने वाले एक
से ज़्यादा स्विच।
