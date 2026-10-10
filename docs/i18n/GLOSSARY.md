# Глоссарий переводов OpenPlane

## Общие правила

- Не переводить: идентификаторы, имена файлов и путей, команды, код, URL, имена режимов (`MANUAL`, `STABILIZE`, `RTH`…), имена функций/крутилок (`FLAPS`, `STAB_GAIN`…), названия плат/чипов/датчиков, `ARM`/`DISARM`, `failsafe` в коде, названия протоколов (iBUS, MAVLink, UBX).
- Числа, десятичная точка, единицы: оставлять Латиницей и теми же значениями: Hz, ms, µs, s, m, m/s, MB, KB, GB, °, %. Русские «Гц, мс, мкс, с, м, м/с, МБ, КБ, ГБ» → Hz, ms, µs, s, m, m/s, MB, KB, GB.
- Текст, который прошивка печатает по-русски (консоль, события чёрного ящика, `summary.txt`) — цитировать дословно в `код`/«кавычках» и там, где важно, давать перевод в скобках.
- Абзац в абзац, строка таблицы в строку, пункт в пункт, блок кода в блок кода; уровни заголовков те же. Ссылки и якоря копировать дословно (build.py их перепишет).
- Комментарии в блоках кода переводить, код — нет. Диаграммы из символов: подписи переводить, ширину строк сохранять.
- Кавычки: en “ ” · zh-CN “ ” · es « » · hi “ ” · ar « » · pt-BR “ ” · fr « » (с узкими пробелами) · de „ “ · ja 「」 · ko “ ” · sv ” ” (одинаковые с обеих сторон).
- Обращение: en — безлично/you; es — tú; hi — आप; ar — безлично/вы (أنت в повелительном); pt-BR — você; fr — vous; de — Sie; ja — です・ます; ko — 합니다체 (해요체 не использовать); sv — du; zh-CN — 你 редко, чаще безлично.
- Орфография: en — US; pt — бразильская; es — нейтральная (Латинская Америка/Испания); fr — французская типографика (пробел перед ; : ! ?); de — новая орфография; sv — тире «–» с пробелами, десятичная запятая и пробел перед % в тексте («0,5 m/s», «98 %»), «firmware» не переводится («firmwaren»).

## Термины

| ru | en | zh-CN | es | hi | ar | pt-BR | fr | de | ja | ko | sv |
|---|---|---|---|---|---|---|---|---|---|---|---|
| автопилот | autopilot | 自动驾驶仪 | piloto automático | ऑटोपायलट | الطيار الآلي | piloto automático | pilote automatique | Autopilot | オートパイロット | 오토파일럿 | autopilot |
| полётный контроллер (плата полётника) | flight controller | 飞控（飞行控制器） | controlador de vuelo | फ़्लाइट कंट्रोलर | وحدة التحكم في الطيران | controladora de voo | contrôleur de vol | Flugsteuerung | フライトコントローラー | 비행 컨트롤러 | flygkontroller |
| самолёт (модель) | airplane | 飞机（固定翼） | avión | हवाई जहाज़ | الطائرة | avião | avion | Flugzeug | 飛行機 | 비행기 | flygplan |
| борт | the aircraft (onboard) | 机上/飞机 | la aeronave / a bordo | विमान | الطائرة / على متنها | a aeronave | l’appareil / à bord | das Fluggerät / an Bord | 機体 | 기체 | planet / ombord |
| планер (конструкция) | airframe | 机体 | célula (airframe) | एयरफ़्रेम | هيكل الطائرة | fuselagem/estrutura (airframe) | cellule | Flugzeugzelle | 機体 | 기체 | flygplansstomme |
| фюзеляж / крыло / шасси | fuselage / wing / landing gear | 机身 / 机翼 / 起落架 | fuselaje / ala / tren de aterrizaje | फ़्यूज़लेज / पंख / लैंडिंग गियर | جسم الطائرة / الجناح / عجلات الهبوط | fuselagem / asa / trem de pouso | fuselage / aile / train d’atterrissage | Rumpf / Tragfläche / Fahrwerk | 胴体 / 主翼 / 降着装置（脚） | 동체 / 날개 / 랜딩기어 | flygkropp / vinge / landningsställ |
| пульт (RC) | transmitter (radio) | 遥控器 | emisora | रिमोट (RC ट्रांसमीटर) | جهاز التحكم عن بعد | rádio (transmissor) | radio (émetteur) | Sender | 送信機 | 조종기 | sändare (radiosändare) |
| приёмник | receiver | 接收机 | receptor | रिसीवर | المستقبل | receptor | récepteur | Empfänger | 受信機 | 수신기 | mottagare |
| стик | stick | 摇杆 | stick | स्टिक | عصا التحكم | stick | stick | Knüppel | スティック | 스틱 | spak |
| тумблер | switch | 开关（拨杆） | interruptor | स्विच | مفتاح | chave | interrupteur | Schalter | スイッチ | 스위치 | brytare |
| крутилка | knob (potentiometer) | 旋钮 | potenciómetro (perilla) | नॉब | مقبض دوّار | potenciômetro (knob) | potentiomètre (bouton) | Drehregler | ダイヤル | 노브 | ratt (potentiometer) |
| сервопривод / серво | servo | 舵机 | servo | सर्वो | سيرفو | servo | servo | Servo | サーボ | 서보 | servo |
| регулятор хода (ESC) | ESC | 电调（ESC） | variador (ESC) | ESC (स्पीड कंट्रोलर) | متحكم السرعة (ESC) | controlador de velocidade (ESC) | contrôleur de vitesse (ESC) | Fahrtregler (ESC) | ESC（スピードコントローラー） | 변속기(ESC) | fartreglage (ESC) |
| газ | throttle | 油门 | acelerador | थ्रॉटल | الخانق (الثروتل) | acelerador | gaz | Gas | スロットル | 스로틀 | gas |
| мотор / винт | motor / propeller | 电机 / 螺旋桨 | motor / hélice | मोटर / प्रोपेलर | المحرك / المروحة | motor / hélice | moteur / hélice | Motor / Propeller | モーター / プロペラ | 모터 / 프로펠러 | motor / propeller |
| рули (поверхности) | control surfaces | 舵面 | superficies de control | कंट्रोल सरफ़ेस | أسطح التحكم | superfícies de comando | gouvernes | Ruder | 舵面 | 조종면 | roderytor |
| элерон / руль высоты / руль направления | aileron / elevator / rudder | 副翼 / 升降舵 / 方向舵 | alerón / timón de profundidad / timón de dirección | एलेरॉन / एलिवेटर / रडर | الجنيح / دفة الارتفاع / دفة الاتجاه | aileron / profundor / leme | aileron / gouverne de profondeur / gouverne de direction | Querruder / Höhenruder / Seitenruder | エルロン / 昇降舵 / 方向舵 | 에일러론 / 승강타 / 방향타 | skevroder / höjdroder / sidroder |
| закрылки / флапероны | flaps / flaperons | 襟翼 / 襟副翼 | flaps / flaperones | फ़्लैप / फ़्लैपरॉन | الفلاب / الفلابرون | flaps / flaperons | volets / flaperons | Klappen / Flaperons | フラップ / フラッペロン | 플랩 / 플래퍼론 | klaffar / flaperoner |
| воздушный тормоз | air brake | 空气刹车（减速板） | aerofreno | एयर ब्रेक | مكبح هوائي | freio aerodinâmico | aérofrein | Bremsklappe | エアブレーキ | 에어브레이크 | luftbroms |
| крен / тангаж / рыскание | roll / pitch / yaw | 横滚 / 俯仰 / 偏航 | alabeo / cabeceo / guiñada | रोल / पिच / यॉ | الدوران الجانبي (Roll) / الميل (Pitch) / الانعراج (Yaw) | rolagem / arfagem / guinada | roulis / tangage / lacet | Roll / Nick / Gier | ロール / ピッチ / ヨー | 롤 / 피치 / 요 | roll / tippning / gir |
| ARM (заармить) | ARM (to arm) | 解锁（ARM） | ARM (armar) | ARM (आर्म करना) | ARM (تسليح) | ARM (armar) | ARM (armer) | ARM (armen) | ARM（アーム） | ARM(시동) | ARM (armera) |
| DISARM | DISARM (to disarm) | 上锁（DISARM） | DISARM (desarmar) | DISARM (डिसआर्म करना) | DISARM (نزع التسليح) | DISARM (desarmar) | DISARM (désarmer) | DISARM (disarmen) | DISARM（ディスアーム） | DISARM(시동 해제) | DISARM (desarmera) |
| failsafe | failsafe | 失控保护（failsafe） | failsafe | फ़ेलसेफ़ | وضع الأمان (failsafe) | failsafe | failsafe | Failsafe | フェイルセーフ | 페일세이프 | failsafe |
| потеря связи | signal loss / link loss | 信号丢失 | pérdida de señal | सिग्नल खो जाना | فقدان الإشارة | perda de sinal | perte de signal | Signalverlust | 信号喪失 | 신호 상실 | förlorad förbindelse |
| возврат домой (RTH) | return to home (RTH) | 返航（RTH） | regreso a casa (RTH) | होम पर वापसी (RTH) | العودة إلى نقطة الانطلاق (RTH) | retorno ao ponto de partida (RTH) | retour au point de départ (RTH) | Rückkehr zum Startpunkt (RTH) | 帰還（RTH） | 귀환(RTH) | hemflygning (RTH) |
| дом (точка) | home (point) | 返航点（Home） | punto de origen (home) | होम पॉइंट | نقطة الانطلاق (Home) | ponto de origem (home) | point de départ (home) | Startpunkt (Home) | ホーム地点 | 홈 지점 | hempunkt |
| кружить (LOITER) | loiter | 盘旋 | orbitar | चक्कर लगाना | التحليق الدائري | orbitar | tourner en orbite | kreisen | 旋回 | 선회 | cirkla (LOITER) |
| геозабор | geofence | 地理围栏 | geovalla | जियोफ़ेंस | السياج الجغرافي | geofence (cerca geográfica) | géorepérage (geofence) | Geofence | ジオフェンス | 지오펜스 | geofence |
| парение / термики | soaring / thermals | 翱翔 / 热气流 | vuelo a vela (soaring) / térmicas | सोअरिंग / थर्मल | التحليق الشراعي / التيارات الحرارية | voo planado (soaring) / térmicas | vol à voile (soaring) / thermiques | Segelflug (Soaring) / Thermik | ソアリング / サーマル | 소어링 / 서멀 | termikflygning / termik |
| сваливание | stall | 失速 | pérdida de sustentación (stall) | स्टॉल | الانهيار الهوائي (Stall) | estol (stall) | décrochage | Strömungsabriss (Stall) | 失速 | 실속 | överstegring |
| взлёт / посадка | takeoff / landing | 起飞 / 降落 | despegue / aterrizaje | टेकऑफ़ / लैंडिंग | الإقلاع / الهبوط | decolagem / pouso | décollage / atterrissage | Start / Landung | 離陸 / 着陸 | 이륙 / 착륙 | start / landning |
| запуск с руки | hand launch | 手抛起飞 | lanzamiento a mano | हाथ से लॉन्च | الإطلاق باليد | lançamento manual | lancement à la main | Handstart | 手投げ発進 | 손으로 던져 이륙 | handstart |
| сброс груза | payload drop | 载荷投放 | lanzamiento de carga | पेलोड ड्रॉप | إسقاط الحمولة | lançamento de carga | largage de charge | Lastabwurf | ペイロード投下 | 화물 투하 | lastsläpp |
| автотриммер / триммер | auto-trim / trim | 自动配平 / 配平 | autotrim / trim | ऑटो-ट्रिम / ट्रिम | الضبط التلقائي (Auto-trim) / التريم | auto-trim / trim | auto-trim / trim | Auto-Trimm / Trimmung | オートトリム / トリム | 오토 트림 / 트림 | autotrimning / trim |
| трубка Пито | pitot tube | 皮托管（空速管） | tubo de Pitot | पिटो ट्यूब | أنبوب بيتو | tubo de Pitot | tube de Pitot | Pitotrohr | ピトー管 | 피토관 | pitotrör |
| воздушная скорость | airspeed | 空速 | velocidad aerodinámica (airspeed) | एयरस्पीड (हवाई गति) | السرعة الجوية | velocidade do ar (airspeed) | vitesse air | Fluggeschwindigkeit (Airspeed) | 対気速度 | 대기속도 | lufthastighet |
| путевая скорость | ground speed | 地速 | velocidad sobre el suelo | ग्राउंड स्पीड | السرعة الأرضية | velocidade em relação ao solo | vitesse sol | Geschwindigkeit über Grund | 対地速度 | 대지속도 | markhastighet |
| барометр | barometer | 气压计 | barómetro | बैरोमीटर | مقياس الضغط (بارومتر) | barômetro | baromètre | Barometer | 気圧センサー | 기압계 | barometer |
| компас (магнитометр) | compass (magnetometer) | 电子罗盘（磁力计） | brújula (magnetómetro) | कम्पास (मैग्नेटोमीटर) | البوصلة (مقياس المغنطيسية) | bússola (magnetômetro) | boussole (magnétomètre) | Kompass (Magnetometer) | コンパス（磁気センサー） | 나침반(지자기 센서) | kompass (magnetometer) |
| IMU / гироскоп / акселерометр | IMU / gyroscope / accelerometer | IMU / 陀螺仪 / 加速度计 | IMU / giroscopio / acelerómetro | IMU / जाइरोस्कोप / एक्सेलेरोमीटर | IMU / الجيروسكوب / مقياس التسارع | IMU / giroscópio / acelerômetro | IMU / gyroscope / accéléromètre | IMU / Gyroskop / Beschleunigungssensor | IMU / ジャイロ / 加速度センサー | IMU / 자이로스코프 / 가속도계 | IMU / gyroskop / accelerometer |
| датчик | sensor | 传感器 | sensor | सेंसर | حساس | sensor | capteur | Sensor | センサー | 센서 | sensor |
| драйвер (датчика) | driver | 驱动 | controlador (driver) | ड्राइवर | مشغّل (درايفر) | driver | pilote (driver) | Treiber | ドライバー | 드라이버 | drivrutin |
| шина (I2C/SPI) | bus | 总线 | bus | बस | ناقل (Bus) | barramento | bus | Bus | バス | 버스 | buss |
| регистр | register | 寄存器 | registro | रजिस्टर | سجل (Register) | registrador | registre | Register | レジスタ | 레지스터 | register |
| датащит | datasheet | 数据手册 | hoja de datos | डेटाशीट | ورقة البيانات | datasheet | fiche technique | Datenblatt | データシート | 데이터시트 | datablad |
| ШИМ | PWM | PWM | PWM | PWM | PWM | PWM | PWM | PWM | PWM | PWM | PWM |
| ПИД | PID | PID | PID | PID | PID | PID | PID | PID | PID | PID | PID |
| такт (цикл управления) | tick (control cycle) | 控制周期 | ciclo de control | कंट्रोल साइकल | دورة التحكم | ciclo de controle | cycle de contrôle | Regelzyklus (Takt) | 制御周期 | 제어 주기 | cykel (styrcykel) |
| полётный контур / замкнутый контур | control loop / closed loop | 控制回路 / 闭环 | lazo de control / lazo cerrado | कंट्रोल लूप / क्लोज़्ड-लूप | حلقة التحكم / الحلقة المغلقة | malha de controle / malha fechada | boucle de contrôle / boucle fermée | Regelkreis / geschlossener Regelkreis | 制御ループ / クローズドループ | 제어 루프 / 폐루프 | styrslinga / sluten slinga |
| конечный автомат | state machine | 状态机 | máquina de estados | स्टेट मशीन | آلة الحالات | máquina de estados | machine à états | Zustandsautomat | ステートマシン | 상태 머신 | tillståndsmaskin |
| чёрный ящик | black box | 黑匣子 | caja negra | ब्लैक बॉक्स | الصندوق الأسود | caixa-preta | boîte noire | Blackbox | ブラックボックス | 블랙박스 | svart låda |
| прошивка | firmware | 固件 | firmware | फ़र्मवेयर | البرنامج الثابت (firmware) | firmware | firmware | Firmware | ファームウェア | 펌웨어 | firmware |
| плата | board | 开发板 / 电路板 | placa | बोर्ड | اللوحة | placa | carte | Platine (Board) | ボード | 보드 | kort |
| стенд / на столе | test bench / on the bench | 台架 / 台架测试 | banco de pruebas | टेस्ट बेंच | منصة الاختبار | bancada de testes | banc d’essai | Prüfstand | テストベンチ | 테스트 벤치 | testbänk / på bänken |
| симуляция | simulation | 仿真 | simulación | सिमुलेशन | المحاكاة | simulação | simulation | Simulation | シミュレーション | 시뮬레이션 | simulering |
| автотесты | automated tests | 自动化测试 | pruebas automáticas | स्वचालित टेस्ट | الاختبارات الآلية | testes automatizados | tests automatiques | automatisierte Tests | 自動テスト | 자동화 테스트 | automatiska tester |
| покрытие (кода) | (code) coverage | 代码覆盖率 | cobertura de código | कोड कवरेज | تغطية الشيفرة | cobertura de código | couverture de code | Codeabdeckung | コードカバレッジ | 코드 커버리지 | (kod)täckning |
| статический анализ | static analysis | 静态分析 | análisis estático | स्टैटिक विश्लेषण | التحليل الساكن | análise estática | analyse statique | statische Analyse | 静的解析 | 정적 분석 | statisk analys |
| сборка (build) | build | 构建 | compilación (build) | बिल्ड | البناء (build) | build (compilação) | compilation (build) | Build | ビルド | 빌드 | bygge |
| залить прошивку | flash / upload | 烧录 / 上传 | cargar el firmware | फ़र्मवेयर फ़्लैश करना | رفع البرنامج الثابت | gravar o firmware | téléverser le firmware | Firmware aufspielen | ファームウェアを書き込む | 펌웨어 업로드 | flasha / ladda upp |
| монитор порта | serial monitor | 串口监视器 | monitor serie | सीरियल मॉनिटर | مراقب المنفذ التسلسلي | monitor serial | moniteur série | serieller Monitor | シリアルモニター | 시리얼 모니터 | seriemonitor |
| консоль | console | 控制台 | consola | कंसोल | وحدة التحكم النصية (الكونسول) | console | console | Konsole | コンソール | 콘솔 | konsol |
| веб-дашборд | web dashboard | 网页仪表盘 | panel web (dashboard) | वेब डैशबोर्ड | لوحة المعلومات عبر الويب | painel web (dashboard) | tableau de bord web | Web-Dashboard | Webダッシュボード | 웹 대시보드 | webbpanel |
| телеметрия | telemetry | 遥测 | telemetría | टेलीमेट्री | القياس عن بُعد (التليمتري) | telemetria | télémétrie | Telemetrie | テレメトリー | 텔레메트리 | telemetri |
| наземная станция | ground station | 地面站 | estación de tierra | ग्राउंड स्टेशन | محطة التحكم الأرضية | estação de solo | station sol | Bodenstation | 地上局 | 지상국 | markstation |
| аккумулятор | battery | 电池 | batería | बैटरी | البطارية | bateria | batterie | Akku | バッテリー | 배터리 | batteri |
| разъём / гребёнка | connector / pin header | 接口 / 排针 | conector / tira de pines | कनेक्टर / पिन हेडर | موصل / شريط دبابيس | conector / barra de pinos | connecteur / barrette | Stecker / Stiftleiste | コネクター / ピンヘッダー | 커넥터 / 핀 헤더 | kontakt / stiftlist |
| SD-карта | SD card | SD 卡 | tarjeta SD | SD कार्ड | بطاقة SD | cartão SD | carte SD | SD-Karte | SDカード | SD 카드 | SD-kort |
| флеш | flash memory | 闪存 | memoria flash | फ़्लैश मेमोरी | ذاكرة الفلاش | memória flash | mémoire flash | Flash-Speicher | フラッシュメモリ | 플래시 메모리 | flashminne |
| задача (FreeRTOS) | task | 任务 | tarea | टास्क | مهمة | tarefa | tâche | Task | タスク | 태스크 | uppgift |
| очередь | queue | 队列 | cola | क्यू | قائمة انتظار | fila | file d’attente | Warteschlange | キュー | 큐 | kö |
| пищалка | buzzer | 蜂鸣器 | zumbador | बजर | صافرة (بازر) | buzzer | buzzer | Summer | ブザー | 부저 | summer |
| 3D-печать | 3D printing | 3D 打印 | impresión 3D | 3D प्रिंटिंग | الطباعة ثلاثية الأبعاد | impressão 3D | impression 3D | 3D-Druck | 3Dプリント | 3D 프린팅 | 3D-utskrift |
| дорожная карта | roadmap | 路线图 | hoja de ruta | रोडमैप | خارطة الطريق | roteiro (roadmap) | feuille de route | Roadmap | ロードマップ | 로드맵 | färdplan |
| инвесторы / партнёры | investors / partners | 投资人 / 合作伙伴 | inversores / socios | निवेशक / साझेदार | المستثمرون / الشركاء | investidores / parceiros | investisseurs / partenaires | Investoren / Partner | 投資家 / パートナー | 투자자 / 파트너 | investerare / partner |
| разработчик / пилот | developer / pilot | 开发者 / 飞手 | desarrollador / piloto | डेवलपर / पायलट | المطوّر / الطيار | desenvolvedor / piloto | développeur / pilote | Entwickler / Pilot | 開発者 / パイロット | 개발자 / 파일럿 | utvecklare / pilot |
| честно / честный статус | honest / honest status | 如实说明 / 真实现状 | con honestidad / estado real | ईमानदारी से / असली स्थिति | بصراحة / الوضع الحقيقي | com honestidade / situação real | en toute franchise / état réel | ehrlich / ehrlicher Stand | 正直なところ / 現状 | 솔직히 / 솔직한 현황 | ärligt talat / ärlig status |
| ограничения | limitations | 局限 | limitaciones | सीमाएँ | القيود | limitações | limitations | Einschränkungen | 制限事項 | 제한 사항 | begränsningar |
