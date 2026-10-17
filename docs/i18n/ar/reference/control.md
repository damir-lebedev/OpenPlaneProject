<div dir="rtl">

# ‏CONTROL وCOORDINATION — الخلّاط والخانق وARM والمخارج والمنسِّق

> 🌐 هذه الصفحة ترجمة لـ[الأصل الروسي](../../../reference/control.md). إذا اختلفت الترجمة عن الأصل فالعبرة بالأصل. يعرض البرنامج الثابت رسائل وحدة التحكم (الكونسول) بالروسية، لذلك تُقتبس هنا كما هي. أُنجزت الترجمة بالذكاء الاصطناعي ولم يراجعها ناطقون أصليون. إذا وجدت أخطاءً فراسل [Damir Lebedev](https://github.com/damir-lebedev) أو أبلغ عنها في [متتبّع المشكلات](https://github.com/damir-lebedev/OpenPlaneProject/issues).

[→ المرجع](README.md)

طبقة CONTROL منطق فوق البيانات، دون UART أو PWM أو Wi-Fi. و`FlightController`
(‏COORDINATION) هو الصنف الوحيد الذي يجمع الطبقات السفلى كلها في دورة واحدة.

---

## ‏`ControlCommand`

**الملف:** `control/ControlCommand.h` · **النوع:** struct

أمر الأسطح بـ**إشارات فيزيائية**، والانحراف بوحدة µs (±500 = الشوط الكامل).
وهو اللغة المشتركة للعصي والطيار الآلي والخلّاط.

| الحقل | معنى «+» |
|---|---|
| `int16_t roll` | دوران جانبي إلى اليمين (الجنيح الأيمن إلى أعلى والأيسر إلى أسفل) |
| `int16_t pitch` | الأنف إلى أعلى (الرافعة إلى أعلى) |
| `int16_t yaw` | الأنف إلى اليمين (الدفة والعجلة إلى اليمين) |
| `int16_t flaps` | الفلابات إلى أسفل (كلا الجنيحين إلى أسفل)؛ و«−» — مكبح هوائي (كلاهما إلى أعلى) |

جميع الحقول افتراضيًّا 0.

---

## ‏`FlightOutputState`

**الملف:** `control/FlightOutputState.h` · **النوع:** struct

نبضات المخارج المطلوبة، بوحدة PWM µs. الافتراضي — الأسطح محايدة والخانق `PWM_MIN`.

‏`throttle` هو مقدار الخانق (1000..2000 = 0..100%) في أي اتجاه؛ ويحوّله `FlightOutputs` مع `motorReverse` إلى نبضة ESC ‏(عبر `EscSignal`).

| الحقل | الافتراضي |
|---|---|
| `aileronLeft` و`aileronRight` و`elevator` و`rudder` | `PWM_CENTER` |
| `throttle` | `PWM_MIN` |
| `bool motorReverse` | `false` — المحرك إلى الأمام |
| `aux1` | `PAYLOAD_CLOSED_US` — جهاز إسقاط الحمولة مغلق |
| `aux2` | `PWM_CENTER` — الكاميرا |

---

## ‏`FlapsController`

**الملف:** `control/FlapsController.h` · **يعتمد على:** `Config`

إخراج الفلابات وسحبها بسلاسة: يتحرك الوضع نحو الهدف (أي قيمة — فلابات من
المفتاح، أو من المقبض، أو مكبح هوائي إلى أعلى) بسرعة لا تزيد على الشوط الكامل
`FLAPS_DEPLOYED_US` خلال `FLAPS_TRANSITION_MS`. ويُمرَّر الزمن معاملًا.

| الدالة | الوصف |
|---|---|
| `int16_t update(float targetUs, uint32_t nowMs)` | خطوة نحو الهدف؛ وتُعيد الوضع الحالي بوحدة µs (+ إلى أسفل، و− إلى أعلى) |
| `int16_t getPosition() const` | الوضع الحالي |

الثوابت:

- **أول استدعاء** يضع الوضع في الهدف مباشرةً — فلا «تخرج» الفلابات
  على المكتب عند التشغيل.
- خطوة الزمن محدودة بـ `MAX_STEP_MS = 20`: فبعد توقف طويل (failsafe، معايرة)
  لا تقفز الفلابات إلى الهدف في دورة واحدة.

---

## ‏`ControlMixer`

**الملف:** `control/ControlMixer.h` · **يعتمد على:** `RcInput` و`RcChannelState` و`FlapsController` و`ControlCommand` و`FlightOutputState` و`Config` و`Channels`

المنطق الأيروديناميكي على خطوتين. ويملك `FlapsController`.

| الدالة | الوصف |
|---|---|
| `ControlCommand fromSticks(const RcChannelState& rc) const` | CH1 ← `roll` (2000 = إلى اليمين)؛ وCH2 ← `pitch` **بإشارة معكوسة** (2000 = بعيدًا عنك = الأنف إلى أسفل)؛ وCH4 ← `yaw` |
| `int16_t updateFlaps(float targetUs, uint32_t nowMs)` | يختار `FlightController` هدف الفلابات (المكبح ← مفتاح الفلابات ← المقبض)، وهنا — الشوط السلس |
| `FlightOutputState mix(const ControlCommand& c) const` | الأمر ← PWM. يُقيَّد الدوران الجانبي والميل والانعراج بالشوط (`*_MAX_US`)؛ والجنيحات: الأيسر = `flaps + roll`، والأيمن = `flaps − roll` (إلى أسفل = «+»)؛ و PWM = `1500 ± الانحراف` بإشارة من `Config::*_REVERSED`، وإشارة الفلابات من `FLAPS_REVERSED`، مع تقييد إلى 1000..2000. ولا يُملأ `throttle` |
| `int16_t getFlaps() const` | وضع الفلابات الحالي، بوحدة µs |

الفلابرون: عند الإخراج ينخفض كلا الجنيحين بمقدار `FLAPS_DEPLOYED_US` («المحايد»
الجديد)، ويعمل الدوران الجانبي فوق ذلك. وعند الدوران الكامل يصل الجنيح
الهابط إلى نهاية شوطه قبل الصاعد — وهذا يعمل كتفاضل للجنيحات.

---

## ‏`ThrottleManager`

**الملف:** `control/ThrottleManager.h` · **يعتمد على:** `RcInput` و`RcChannelState` و`Config` و`Channels`

| الدالة | الوصف |
|---|---|
| `uint16_t update(const RcChannelState& rc, bool receiverFailsafe) const` | خانق CH3 مقيَّدًا إلى 1000..2000؛ وعند فقدان الاتصال — `FAILSAFE_THROTTLE` |

لا يعرف شيئًا عن ARM والطيار الآلي — فتصحيحاتهما يطبّقها `FlightController`.

---

## ‏`EscSignal`

**الملف:** `control/EscSignal.h` · **النوع:** namespace · **يعتمد على:** `Config`

الخانق (1000..2000 = 0..100%) والاتجاه ← النبضة على طرف ESC.

| الدالة | الوصف |
|---|---|
| `uint16_t pulseUs(uint16_t throttleUs, bool reverse)` | ‏`ESC_BIDIRECTIONAL` ‏(BLHeli 80A في وضع 3D): 0% ← `ESC_NEUTRAL_US` ‏(1500)، وإلى الأمام 1500..2000، وإلى الخلف 1500..1000. والمتحكم أحادي الاتجاه: 1000..2000 كما هي، ويُتجاهل `reverse`. ويُقصّ الخانق خارج 1000..2000 |
| `uint16_t stopUs()` | نبضة «المحرك متوقف» (1500 في ثنائي الاتجاه، و1000 في العادي) |

تحويل فقط بلا حمايات: متى يُبدَّل الاتجاه يقرّره `MotorReverse`، وحدّ الخانق إلى الخلف — `FlightController`.

---

## ‏`MotorReverse`

**الملف:** `control/MotorReverse.h` · **يعتمد على:** `Config`

اتجاه المحرك بحسب مفتاح `Feature::MOTOR_REVERSE` (افتراضيًّا SwD) — **دون ARM فقط**.

| الدالة | الوصف |
|---|---|
| `void update(bool requested, bool armed, uint32_t nowMs)` | دون ARM — يتبع الاتجاه المفتاحَ فورًا (يُطبع في Serial ‏`MotorReverse: мотор НАЗАД` / `вперёд` — «المحرك إلى الخلف» / «إلى الأمام»). وأثناء ARM — لا أثر للمفتاح، ويُطبع مرة واحدة «реверс НЕ переключён» («لم يُبدَّل العكس»)، وينتظر الاتجاه الجديد DISARM. ومع `ESC_BIDIRECTIONAL = false` لا يُفعَّل العكس أبدًا |
| `bool isReversed() const` | المحرك إلى الخلف |
| `bool isPending() const` | المفتاح في الوضع الآخر فعلًا، والاتجاه ينتظر DISARM |
| `const char* armBlockReason(uint32_t nowMs) const` | سبب لرفض ARM: مرّ على آخر تبديل للاتجاه أقل من `REVERSE_SETTLE_MS` (قد يكون المحرك ما زال يدور)؛ وإلا `nullptr` |

---

## ‏`ArmingManager`

**الملف:** `control/ArmingManager.h` · **يعتمد على:** `Autopilot` (يمكن أن يكون فارغًا) و`RcChannelState` و`Config` و`Channels`

التفعيل ARM بمفتاح مستقل SwA (‏CH5). وآلة الحالات في
[ARCHITECTURE.md §7](../ARCHITECTURE.md#arm-armingmanager).

| الدالة | الوصف |
|---|---|
| `explicit ArmingManager(Autopilot* ap = nullptr)` | دون طيار آلي يُفحص الخانق فقط |
| `void update(const RcChannelState& rc, bool receiverFailsafe, const char* blockReason = nullptr)` | عند failsafe — لا شيء (فالمفتاح في إطار failsafe لا يعكس الطيار). المفتاح OFF ← DISARM، و`switchSeenOff = true`. والانتقال OFF→ON ← الفحوص ← ARM أو رفض و`blockReason` سبب رفض خارجي (من `FlightController`: ‏`MotorReverse::armBlockReason()`) |
| `bool isArmed() const` | مُفعَّل |
| `const char* getLastRefusalReason() const` | سبب آخر رفض أو `nullptr`؛ ويُمسح عند إطفاء المفتاح |

‏`checkFailureReason(rc)` — فحوص ARM:

| الشرط | سبب الرفض |
|---|---|
| الخانق ≥ `THROTTLE_LOW_US` | «الخانق ليس عند الحد الأدنى» |
| أي وضع عدا MANUAL، وIMU موجودة لكنها لا تستجيب | «IMU لا تستجيب…» |
| أي وضع عدا MANUAL، وفي IMU مشكلة بفحص ما قبل الطيران | نص `ImuSensor::getPreflightProblem()` |
| وضع يتطلب الارتفاع (`needsAltitude`: ‏ALT_HOLD وCRUISE وLOITER وRTH وAUTO_LAND وSOARING)، ومقياس الضغط موجود لكنه لا يستجيب | «مقياس الضغط لا يستجيب…» |
| نجح كل ما عدا ذلك، لكن مُرِّر `blockReason` | `blockReason` (مثلًا «تبدّل اتجاه المحرك للتو…») |

الحساس غير الموجود في البناء (`nullptr`) لا يمنع ARM؛ وفي MANUAL تُفعَّل
الطائرة حتى دون أي حساسات. ولم يُدرج تثبيت GPS في الفحوص عمدًا: فدون GPS
تتصرف أوضاع الملاحة بأمان (دائرة في المكان)، وتُسجَّل نقطة الانطلاق عندما
يلتقط GPS الأقمار.

الثوابت: تشغيل اللوحة والمفتاح في وضع ON لا يفعّل ARM؛ ومحاولة واحدة لكل
انتقال OFF→ON؛ وفقدان الاتصال لا يُلغي ARM.

---

## ‏`FlightOutputs`

**الملف:** `control/FlightOutputs.h` · **يعتمد على:** `IBoard` و`FlightOutputState` و`EscSignal` و`Config`

الصنف الوحيد الذي يعرف مجموعة مخارج PWM وترتيبها. وجميع المخارج موصوفة في
جدول واحد؛ وتمرّ عليه `begin()` و`write()` والحالة والفحص الذاتي في حلقة.

### ‏`FlightOutputs::OutputInfo`

| الحقل | الوصف |
|---|---|
| `const char* key` | الاسم في JSON/السجل (`aileronLeft` و… و`esc` و`rudder` و`aux1` و`aux2`) |
| `const char* label` | الاسم المقروء للبشر |
| `int16_t pin` | رقم الطرف؛ و`-1` — غير موصول. و`int16_t` لأن أرقام الأطراف التناظرية في STM32 هي `0xC0 + N` |
| `bool required` | دونه لا تطير الطائرة (الدفة اختيارية) |
| `uint16_t FlightOutputState::* field` | مؤشر إلى حقل الحالة |

| الدالة | الوصف |
|---|---|
| `static const OutputInfo& outputInfo(uint8_t ch)` | سطر من الجدول؛ والترتيب = `ServoChannel` |
| `explicit FlightOutputs(IBoard& board)` | |
| `bool begin()` | ‏`attach(PWM_MIN, PWM_MAX)` لكل مخرج، وطباعة الحالة ونوع ESC؛ و`true` إن حصلت كل المخارج **الإلزامية** على قناة |
| `bool isAttached(uint8_t ch) const` | المخرج موصول (فهرس خارج المدى ← `false`) |
| `static uint16_t valueOf(const FlightOutputState&, uint8_t ch)` | النبضة على طرف المخرج: الحقل بحسب الجدول، وللـ ESC — عبر `EscSignal::pulseUs(throttle, motorReverse)` |
| `void printStatus() const` | `Outputs: aileronLeft(GPIO4)=OK …` |
| `void printPulseSelfTest()` | النبضة المقيسة مقابل المتوقعة على كل طرف موصول؛ و«OK» عندما يكون الفرق ≤ 15 µs |
| `void write(const FlightOutputState&)` | كتابة جميع المخارج وتذكّر الحالة |
| `void setFailsafe()` | أسطح التحكم في الوضع المحايد (`FAILSAFE_*`)، والخانق `FAILSAFE_THROTTLE` ‏(1500 على طرف ESC ثنائي الاتجاه)؛ وAUX كما كانت (لا تُسقَط الحمولة بفقدان الاتصال) |
| `void setBuzzer(bool on)` | صافرة اللوحة (`IBoard::setBuzzer`) |
| `const FlightOutputState& getLastState() const` | آخر حالة مكتوبة |

لإضافة مخرج: سطر في الجدول + حقل في `FlightOutputState` + فهرس في
`ServoChannel` (+ طرف وقناة LEDC في `Esp32Board`).

---

## ‏`FlightController`

**الملف:** `control/FlightController.h` · **الطبقة:** COORDINATION ·
**يعتمد على:** `IBusReceiver` و`ControlMixer` و`ThrottleManager` و`ArmingManager` و`FlightOutputs` و`Autopilot*` و`PilotSwitches*` و`Beeper` و`MotorReverse`

المنسِّق الوحيد لحلقة التحكم: لا يحلل UART بنفسه، ولا يمس PWM، ولا يحسب
الخلّاط — بل يستدعي الآخرين بالترتيب الصحيح فقط. والمخطط المفصّل في
[ARCHITECTURE.md §6](../ARCHITECTURE.md#6-دورة-التحكم-flightcontrollerupdate).

| الدالة | الوصف |
|---|---|
| `FlightController(IBusReceiver&, ControlMixer&, ThrottleManager&, ArmingManager&, FlightOutputs&, Autopilot* = nullptr, PilotSwitches* = nullptr)` | دون طيار آلي — تحكم يدوي صرف؛ ودون مفاتيح — العصي فقط |
| `void begin()` | `outputs.setFailsafe()` و`receiver.begin()` |
| `void update()` | دورة واحدة (انظر أدناه) |
| `bool isReceiverFailsafe() const` | الاتصال مفقود |
| `const IBusReceiver& getReceiver() const` | للسجل (عدّادات الإطارات، وسبب الفقدان) |
| `bool isArmed() const` و`const ArmingManager& getArming() const` | ARM |
| `const FlightOutputState& getOutputState() const` | آخر ما كُتب على المخارج |
| `const RcChannelState& getRcState() const` | القنوات |
| `const FlightOutputs& getOutputs() const` | جدول المخارج و`attached` |
| `int16_t getFlapsUs() const` | وضع الفلابات |
| `const PilotSwitches* getSwitches() const` و`const PilotInputs& getInputs() const` | مفاتيح هذه الدورة ومقابضها |
| `bool isLostModelBeeping() const` | صافرة «أنا هنا» تعمل |
| `const MotorReverse& getMotorReverse() const` | اتجاه المحرك والعكس المنتظر |

ترتيب `update()`:

1. ‏`receiver.update()`؛ و`failsafe = receiver.isSignalLost()`؛
2. والاتصال قائم — `switches->update(rc)` (الوضع، والوظائف، والمقابض)؛
3. ‏`pilotThrottle = throttle.update(rc, failsafe)`؛
4. العصي `mixer.fromSticks(rc)` (والاتصال قائم) × `Knob::RATES`؛ والفلابات
   `mixer.updateFlaps(target)`: ‏`AIRBRAKE` ← −`AIRBRAKE_US`، و`FLAPS` ←
   `FLAPS_DEPLOYED_US`، و`Knob::FLAPS` ← بسلاسة، وعند فقدان الاتصال — 0؛
5. ‏`autopilot->update(armed, failsafe, pilotThrottle, sticks)` — **دائمًا**؛
   وفي العكس يُمرَّر `PWM_MIN` بدل `pilotThrottle` (المحرك مطفأ بالنسبة إلى الطيار الآلي)؛
6. الصافرة: `Beeper::update(BEEPER, armed, failsafe, now)`؛
7. الاتصال مفقود ← `applyLinkLoss()` والخروج من الدورة؛
8. ‏`arming.update(rc, false, reverse.armBlockReason(now))`، ثم
   `reverse.update(MOTOR_REVERSE, armed, now)` — عند DISARM يتبدّل الاتجاه في الدورة نفسها؛
9. ‏`command = autopilot->getCommand()` (أو العصي دون طيار آلي)، والفلابات — خاصتها؛
10. ‏`output = mixer.mix(command)`؛ و`output.motorReverse = reverse.isReversed()`؛
    إلى الأمام — `output.throttle = autopilot->applyThrottle(pilotThrottle)` بحدّ
    `THROTTLE_LIMIT_PCT` × المقبض `Knob::THROTTLE_LIMIT`، وإلى الخلف — خانق الطيار بحدّ
    `min(REVERSE_THROTTLE_LIMIT_PCT, الحدّ الأمامي)`؛ ويُمدّ المسموح على كامل شوط العصا؛
11. غير مُفعَّل أو `MOTOR_KILL` ← `throttle = PWM_MIN` (آخر خطوة)؛
12. ‏AUX1 — الحمولة (`PAYLOAD_DROP`)، وAUX2 — الكاميرا (`Knob::CAMERA_TILT`، و`CAMERA_STAB` يطرح الميل)؛ وتعكس `AUX1_REVERSED`/`AUX2_REVERSED` النبضة.
13. ‏`outputs.write(output)`.

‏`applyLinkLoss()`: إذا كان الطيار الآلي في failsafe (مُفعَّل: RTH أو
انسياب) — فالأسطح والخانق وفق أمر الطيار الآلي (تُسحب الفلابات بسلاسة،
و`MOTOR_KILL` ما زال يُسكت المحرك، وAUX كما كانت)؛ وإلا فـ`outputs.setFailsafe()`. وفي العكس يبقى المحرك متوقفًا دائمًا.

---

## ‏`Beeper`

**الملف:** `control/Beeper.h` · **يعتمد على:** `Config`

| الدالة | الوصف |
|---|---|
| `bool update(bool requested, bool armed, bool linkLost, uint32_t nowMs)` | حالة الصافرة: بتردد 2 Hz إذا كانت `Feature::BEEPER` أو «النموذج مفقود» (غير مُفعَّل، ولا اتصال لمدة أطول من `LOST_MODEL_BEEP_DELAY_MS`) |
| `bool isLostModel() const` | وضع «ابحث عني في العشب» |

</div>
