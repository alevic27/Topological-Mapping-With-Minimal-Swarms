15/4/24: incontro del gruppo 
note importanti: gym-pybullet-drones e la sua compatibilità con betaflight sono stati aggiornati.
Tentativo di reinstall nella cartella

~/Desktop/CFRRS2_Apr_24

##########################################################

Riprovo oggi 23/4 da capo. eliminato tutto nella cartella CFRRS2_Apr_24 a parte 'pycfirmware' in quanto è un software che potrebbe essere utile.

reinstalazione di gym-pybuttet-drones seguendo il suo readme senza creare l'ambiente 'drones' perché già presente. 
apro terminale in CFRRS2_Apr_24 e runno:

$ conda activate drones
$ git clone https://github.com/utiasDSL/gym-pybullet-drones.git
$ cd gym-pybullet-drones/
$ pip3 install --upgrade pip
$ pip3 install -e .

provo il funzionamento di qualche esempio (stesso terminale):

$ cd gym_pybullet_drones/examples/
$ python3 downwash.py

tutto a posto. Provo un altro:

$ python3 cff-dsl.py

dà problemi perché non esiste un file chiamato cff-dsl.py in examples nonostante questo comando sia nel readme.
Guardando i file in examples provo il più simile:

$ python3 cf.py

gira tutto. l'esempio cff-dsl.py mi interessava perché forse è quello per cui serve pycfirmware.
Per il betaflight example (beta.py) c'è una guida sul readme e un'altra guida nello stesso file. Provo solo quella contenuta nel file.
Ripartendo col terminale in 'gym-pybullet-drones' runno:

$ gym_pybullet_drones/assets/clone_bfs.sh 2

Incollo output qui sotto. Per zomparlo veloce vai alla riga 679 di questo file

Cloning into 'temp'...
remote: Enumerating objects: 216385, done.
remote: Counting objects: 100% (2112/2112), done.
remote: Compressing objects: 100% (876/876), done.
remote: Total 216385 (delta 1374), reused 1811 (delta 1193), pack-reused 214273
Receiving objects: 100% (216385/216385), 390.66 MiB | 2.76 MiB/s, done.
Resolving deltas: 100% (148611/148611), done.
mkdir -p tools
mkdir -p downloads
  % Total    % Received % Xferd  Average Speed   Time    Time     Time  Current
                                 Dload  Upload   Total   Spent    Left  Speed
  0     0    0     0    0     0      0      0 --:--:-- --:--:-- --:--:--     
  100   252  100   252    0     0    268      0 --:--:-- --:--:-- --:--:--   26
  100   252  100   252    0     0    268      0 --:--:-- --:--:-- --:--:--   267
  0  149M    0  287k    0     0   170k      0  0:14:57  0:00:01  0:14:56  170  
  2  149M    2 4224k    0     0  1527k      0  0:01:40  0:00:02  0:01:38 3638  
  4  149M    4 6752k    0     0  1833k      0  0:01:23  0:00:03  0:01:20 3232  
  6  149M    6 9696k    0     0  2070k      0  0:01:14  0:00:04  0:01:10 3136  
  7  149M    7 11.8M    0     0  2139k      0  0:01:11  0:00:05  0:01:06 2968  
  9  149M    9 14.2M    0     0  2186k      0  0:01:10  0:00:06  0:01:04 2864 
  10  149M   10 16.3M    0     0  2184k      0  0:01:10  0:00:07  0:01:03 2553 
  12  149M   12 18.4M    0     0  2174k      0  0:01:10  0:00:08  0:01:02 2425 
  13  149M   13 19.9M    0     0  2113k      0  0:01:12  0:00:09  0:01:03 2154 
  14  149M   14 22.2M    0     0  2137k      0  0:01:11  0:00:10  0:01:01 2134 
  16  149M   16 24.0M    0     0  2098k      0  0:01:13  0:00:11  0:01:02 1982 
  17  149M   17 25.9M    0     0  2094k      0  0:01:13  0:00:12  0:01:01 1955 
  18  149M   18 27.4M    0     0  2056k      0  0:01:14  0:00:13  0:01:01 1852 
  19  149M   19 28.6M    0     0  1999k      0  0:01:16  0:00:14  0:01:02 1779 
  20  149M   20 30.0M    0     0  1959k      0  0:01:18  0:00:15  0:01:03 1580 
  21  149M   21 31.9M    0     0  1957k      0  0:01:18  0:00:16  0:01:02 1625 
  22  149M   22 33.5M    0     0  1941k      0  0:01:18  0:00:17  0:01:01 1555 
  23  149M   23 35.9M    0     0  1969k      0  0:01:17  0:00:18  0:00:59 1732 
  25  149M   25 37.6M    0     0  1960k      0  0:01:18  0:00:19  0:00:59 1843 
  26  149M   26 40.2M    0     0  1994k      0  0:01:16  0:00:20  0:00:56 2100 
  27  149M   27 41.9M    0     0  1980k      0  0:01:17  0:00:21  0:00:56 2056 
  29  149M   29 44.3M    0     0  2003k      0  0:01:16  0:00:22  0:00:54 2223 
  30  149M   30 45.9M    0     0  1984k      0  0:01:17  0:00:23  0:00:54 2041 
  32  149M   32 48.0M    0     0  1992k      0  0:01:17  0:00:24  0:00:53 2118 
  33  149M   33 49.6M    0     0  1978k      0  0:01:17  0:00:25  0:00:52 1914 
  34  149M   34 52.2M    0     0  2000k      0  0:01:16  0:00:26  0:00:50 2088 
  35  149M   35 53.6M    0     0  1984k      0  0:01:17  0:00:27  0:00:50 1894 
  37  149M   37 55.7M    0     0  1989k      0  0:01:17  0:00:28  0:00:49 2012 
  38  149M   38 57.2M    0     0  1976k      0  0:01:17  0:00:29  0:00:48 1899 
  39  149M   39 58.5M    0     0  1953k      0  0:01:18  0:00:30  0:00:48 1823 
  39  149M   39 59.4M    0     0  1920k      0  0:01:19  0:00:31  0:00:48 1484 
  41  149M   41 61.5M    0     0  1928k      0  0:01:19  0:00:32  0:00:47 1622 
  42  149M   42 63.3M    0     0  1924k      0  0:01:19  0:00:33  0:00:46 1551 
  43  149M   43 65.4M    0     0  1931k      0  0:01:19  0:00:34  0:00:45 1660 
  44  149M   44 66.9M    0     0  1920k      0  0:01:19  0:00:35  0:00:44 1715 
  45  149M   45 68.7M    0     0  1918k      0  0:01:19  0:00:36  0:00:43 1901 
  46  149M   46 70.2M    0     0  1909k      0  0:01:20  0:00:37  0:00:43 1779 
  48  149M   48 72.0M    0     0  1904k      0  0:01:20  0:00:38  0:00:42 1768 
  49  149M   49 74.0M    0     0  1910k      0  0:01:20  0:00:39  0:00:41 1766 
  51  149M   51 76.6M    0     0  1928k      0  0:01:19  0:00:40  0:00:39 1990 
  52  149M   52 78.4M    0     0  1927k      0  0:01:19  0:00:41  0:00:38 1993 
  53  149M   53 80.3M    0     0  1928k      0  0:01:19  0:00:42  0:00:37 2073 
  54  149M   54 82.0M    0     0  1922k      0  0:01:19  0:00:43  0:00:36 2061 
  56  149M   56 84.3M    0     0  1933k      0  0:01:19  0:00:44  0:00:35 2115 
  57  149M   57 85.8M    0     0  1923k      0  0:01:19  0:00:45  0:00:34 1883 
  58  149M   58 88.0M    0     0  1928k      0  0:01:19  0:00:46  0:00:33 1941 
  59  149M   59 89.5M    0     0  1922k      0  0:01:19  0:00:47  0:00:32 1874 
  61  149M   61 91.7M    0     0  1930k      0  0:01:19  0:00:48  0:00:31 2000 
  62  149M   62 93.5M    0     0  1927k      0  0:01:19  0:00:49  0:00:30 1872 
  63  149M   63 95.3M    0     0  1926k      0  0:01:19  0:00:50  0:00:29 1954 
  64  149M   64 96.0M    0     0  1899k      0  0:01:20  0:00:51  0:00:29 1633 
  64  149M   64 96.4M    0     0  1873k      0  0:01:21  0:00:52  0:00:29 1412 
  64  149M   64 96.8M    0     0  1842k      0  0:01:23  0:00:53  0:00:30 1010 
  65  149M   65 98.1M    0     0  1837k      0  0:01:23  0:00:54  0:00:29  952 
  66  149M   66 99.8M    0     0  1835k      0  0:01:23  0:00:55  0:00:28  912 
  67  149M   67  101M    0     0  1831k      0  0:01:23  0:00:56  0:00:27 1106 
  68  149M   68  102M    0     0  1825k      0  0:01:24  0:00:57  0:00:27 1308 
  69  149M   69  103M    0     0  1809k      0  0:01:24  0:00:58  0:00:26 1448 
  69  149M   69  104M    0     0  1794k      0  0:01:25  0:00:59  0:00:26 1312 
  70  149M   70  105M    0     0  1784k      0  0:01:25  0:01:00  0:00:25 1210 
  71  149M   71  107M    0     0  1789k      0  0:01:25  0:01:01  0:00:24 1308 
  73  149M   73  109M    0     0  1787k      0  0:01:25  0:01:02  0:00:23 1353 
  74  149M   74  110M    0     0  1783k      0  0:01:26  0:01:03  0:00:23 1475 
  75  149M   75  112M    0     0  1783k      0  0:01:26  0:01:04  0:00:22 1654 
  76  149M   76  115M    0     0  1795k      0  0:01:25  0:01:05  0:00:20 1935 
  78  149M   78  117M    0     0  1797k      0  0:01:25  0:01:06  0:00:19 1907 
  79  149M   79  119M    0     0  1804k      0  0:01:25  0:01:07  0:00:18 2020 
  80  149M   80  120M    0     0  1802k      0  0:01:25  0:01:08  0:00:17 2050 
  81  149M   81  122M    0     0  1799k      0  0:01:25  0:01:09  0:00:16 2011 
  82  149M   82  123M    0     0  1795k      0  0:01:25  0:01:10  0:00:15 1795 
  84  149M   84  126M    0     0  1802k      0  0:01:25  0:01:11  0:00:14 1865 
  85  149M   85  128M    0     0  1804k      0  0:01:25  0:01:12  0:00:13 1804 
  86  149M   86  129M    0     0  1799k      0  0:01:25  0:01:13  0:00:12 1750 
  87  149M   87  130M    0     0  1794k      0  0:01:25  0:01:14  0:00:11 1723 
  88  149M   88  132M    0     0  1784k      0  0:01:25  0:01:15  0:00:10 1623 
  88  149M   88  133M    0     0  1779k      0  0:01:26  0:01:16  0:00:10 1452 
  90  149M   90  135M    0     0  1779k      0  0:01:26  0:01:17  0:00:09 1414 
  91  149M   91  136M    0     0  1777k      0  0:01:26  0:01:18  0:00:08 1449 
  91  149M   91  137M    0     0  1767k      0  0:01:26  0:01:19  0:00:07 1364 
  93  149M   93  139M    0     0  1775k      0  0:01:26  0:01:20  0:00:06 1643 
  94  149M   94  141M    0     0  1773k      0  0:01:26  0:01:21  0:00:05 1676 
  95  149M   95  142M    0     0  1769k      0  0:01:26  0:01:22  0:00:04 1606 
  96  149M   96  144M    0     0  1764k      0  0:01:26  0:01:23  0:00:03 1570 
  96  149M   96  145M    0     0  1753k      0  0:01:27  0:01:24  0:00:03 1533 
  97  149M   97  146M    0     0  1749k      0  0:01:27  0:01:25  0:00:02 1321 
  98  149M   98  148M    0     0  1749k      0  0:01:27  0:01:26  0:00:01 1360 
  99  149M   99  149M    0     0  1746k      0  0:01:27  0:01:27 --:--:-- 1378
  100  149M  100  149M    0     0  1746k      0  0:01:27  0:01:27 --:--:-- 1376k
make -j ./obj/betaflight_4.5.0_SITL.hex
make[1]: Entering directory '/home/alessandro/Desktop/CFRRS2_Apr_24/gym-pybullet-drones/betaflight_sitl/bf0'
rm -f ./obj/main/SITL/.efhash_*
EF HASH -> ./obj/main/SITL/.efhash_d41d8cd98f00b204e9800998ecf8427e
%% (optimised) lib/main/dyad/dyad.c 
%% (optimised) ./src/main/drivers/accgyro/accgyro_virtual.c 
%% (optimised) ./src/main/drivers/barometer/barometer_virtual.c 
%% (optimised) ./src/main/drivers/compass/compass_virtual.c 
%% (size optimised) ./src/main/drivers/serial_tcp.c 
%% (optimised) ./src/main/build/build_config.c 
%% (optimised) ./src/main/build/debug.c 
%% (optimised) ./src/main/build/debug_pin.c 
%% (optimised) ./src/main/build/version.c 
%% (optimised) ./src/main/target/SITL/udplink.c 
%% (optimised) ./src/main/target/SITL/sitl.c 
%% (size optimised) ./src/main/main.c 
%% (optimised) ./src/main/pg/beeper.c 
%% (optimised) ./src/main/pg/adc.c 
%% (optimised) ./src/main/pg/beeper_dev.c 
%% (optimised) ./src/main/pg/board.c 
%% (optimised) ./src/main/pg/bus_i2c.c 
%% (optimised) ./src/main/pg/bus_quadspi.c 
%% (optimised) ./src/main/pg/bus_spi.c 
%% (optimised) ./src/main/pg/dashboard.c 
%% (optimised) ./src/main/pg/displayport_profiles.c 
%% (optimised) ./src/main/pg/dyn_notch.c 
%% (optimised) ./src/main/pg/flash.c 
%% (optimised) ./src/main/pg/gps.c 
%% (optimised) ./src/main/pg/gps_lap_timer.c 
%% (optimised) ./src/main/pg/gps_rescue.c 
%% (optimised) ./src/main/pg/gyrodev.c 
%% (optimised) ./src/main/pg/mco.c 
%% (optimised) ./src/main/pg/max7456.c 
%% (optimised) ./src/main/pg/motor.c 
%% (optimised) ./src/main/pg/msp.c 
%% (optimised) ./src/main/pg/pg.c 
%% (optimised) ./src/main/pg/piniobox.c 
%% (optimised) ./src/main/pg/pinio.c 
%% (optimised) ./src/main/pg/pin_pull_up_down.c 
%% (optimised) ./src/main/pg/rcdevice.c 
%% (optimised) ./src/main/pg/rx.c 
%% (optimised) ./src/main/pg/rx_pwm.c 
%% (optimised) ./src/main/pg/rpm_filter.c 
%% (optimised) ./src/main/pg/rx_spi_cc2500.c 
%% (optimised) ./src/main/pg/rx_spi.c 
%% (optimised) ./src/main/pg/rx_spi_expresslrs.c 
%% (optimised) ./src/main/pg/scheduler.c 
%% (optimised) ./src/main/pg/sdcard.c 
%% (optimised) ./src/main/pg/sdio.c 
%% (optimised) ./src/main/pg/stats.c 
%% (optimised) ./src/main/pg/serial_uart.c 
%% (optimised) ./src/main/pg/timerio.c 
%% (optimised) ./src/main/pg/timerup.c 
%% (optimised) ./src/main/pg/usb.c 
%% (optimised) ./src/main/pg/vcd.c 
%% (optimised) ./src/main/pg/vtx_io.c 
%% (optimised) ./src/main/pg/vtx_table.c 
%% (optimised) ./src/main/common/bitarray.c 
%% (optimised) ./src/main/common/colorconversion.c 
%% (optimised) ./src/main/common/crc.c 
%% (speed optimised) ./src/main/common/encoding.c 
%% (optimised) ./src/main/common/explog_approx.c 
%% (speed optimised) ./src/main/common/filter.c 
%% (optimised) ./src/main/common/gps_conversion.c 
%% (optimised) ./src/main/common/huffman.c 
%% (optimised) ./src/main/common/huffman_table.c 
%% (speed optimised) ./src/main/common/maths.c 
%% (optimised) ./src/main/common/printf.c 
%% (optimised) ./src/main/common/printf_serial.c 
%% (speed optimised) ./src/main/common/sdft.c 
%% (optimised) ./src/main/common/sensor_alignment.c 
%% (speed optimised) ./src/main/common/stopwatch.c 
%% (optimised) ./src/main/common/streambuf.c 
%% (optimised) ./src/main/common/strtol.c 
%% (optimised) ./src/main/common/string_light.c 
%% (optimised) ./src/main/common/time.c 
%% (speed optimised) ./src/main/common/typeconversion.c 
%% (optimised) ./src/main/common/uvarint.c 
%% (optimised) ./src/main/config/config.c 
%% (size optimised) ./src/main/config/config_eeprom.c 
%% (size optimised) ./src/main/config/config_streamer.c 
%% (size optimised) ./src/main/config/simplified_tuning.c 
%% (size optimised) ./src/main/config/feature.c 
%% (size optimised) ./src/main/cli/cli.c 
%% (size optimised) ./src/main/cli/settings.c 
%% (optimised) ./src/main/drivers/dshot.c 
%% (optimised) ./src/main/drivers/dshot_dpwm.c 
%% (optimised) ./src/main/drivers/dshot_command.c 
%% (speed optimised) ./src/main/drivers/buf_writer.c 
%% (speed optimised) ./src/main/drivers/bus.c 
%% (optimised) ./src/main/drivers/bus_i2c_busdev.c 
%% (optimised) ./src/main/drivers/bus_i2c_utils.c 
%% (optimised) ./src/main/drivers/bus_i2c_soft.c 
%% (optimised) ./src/main/drivers/bus_octospi.c 
%% (optimised) ./src/main/drivers/display.c 
%% (optimised) ./src/main/drivers/buttons.c 
%% (speed optimised) ./src/main/drivers/bus_quadspi.c 
%% (speed optimised) ./src/main/drivers/io.c 
%% (optimised) ./src/main/drivers/dma_common.c 
%% (optimised) ./src/main/drivers/display_canvas.c 
%% (optimised) ./src/main/drivers/light_led.c 
%% (optimised) ./src/main/drivers/mco.c 
%% (optimised) ./src/main/drivers/motor.c 
%% (optimised) ./src/main/drivers/pinio.c 
%% (optimised) ./src/main/drivers/resource.c 
%% (optimised) ./src/main/drivers/pin_pull_up_down.c 
%% (speed optimised) ./src/main/drivers/serial.c 
%% (optimised) ./src/main/drivers/stack_check.c 
%% (optimised) ./src/main/drivers/sound_beeper.c 
%% (optimised) ./src/main/drivers/timer_common.c 
%% (optimised) ./src/main/drivers/transponder_ir_arcitimer.c 
%% (optimised) ./src/main/drivers/transponder_ir_erlt.c 
%% (optimised) ./src/main/drivers/transponder_ir_ilap.c 
%% (size optimised) ./src/main/fc/board_info.c 
%% (optimised) ./src/main/fc/dispatch.c 
%% (optimised) ./src/main/fc/hardfaults.c 
%% (speed optimised) ./src/main/fc/tasks.c 
%% (speed optimised) ./src/main/fc/runtime_config.c 
%% (optimised) ./src/main/fc/stats.c 
%% (optimised) ./src/main/io/beeper.c 
%% (optimised) ./src/main/io/piniobox.c 
%% (size optimised) ./src/main/io/serial.c 
%% (optimised) ./src/main/io/smartaudio_protocol.c 
%% (optimised) ./src/main/io/statusindicator.c 
%% (optimised) ./src/main/io/tramp_protocol.c 
%% (size optimised) ./src/main/io/transponder_ir.c 
%% (size optimised) ./src/main/io/usb_cdc_hid.c 
%% (optimised) ./src/main/io/usb_msc.c 
%% (optimised) ./src/main/msp/msp.c 
%% (optimised) ./src/main/msp/msp_build_info.c 
%% (size optimised) ./src/main/msp/msp_serial.c 
%% (optimised) ./src/main/msp/msp_box.c 
%% (speed optimised) ./src/main/scheduler/scheduler.c 
%% (optimised) ./src/main/sensors/adcinternal.c 
%% (optimised) ./src/main/sensors/battery.c 
%% (optimised) ./src/main/sensors/current.c 
%% (optimised) ./src/main/sensors/voltage.c 
%% (size optimised) ./src/main/fc/init.c 
%% (optimised) ./src/main/target/config_helper.c 
%% (optimised) ./src/main/fc/controlrate_profile.c 
%% (optimised) ./src/main/drivers/accgyro/gyro_sync.c 
%% (optimised) ./src/main/drivers/rx/rx_spi.c 
%% (optimised) ./src/main/drivers/rx/rx_pwm.c 
%% (optimised) ./src/main/drivers/serial_softserial.c 
%% (speed optimised) ./src/main/fc/core.c 
%% (speed optimised) ./src/main/fc/rc.c 
%% (optimised) ./src/main/fc/rc_adjustments.c 
%% (speed optimised) ./src/main/fc/rc_controls.c 
%% (optimised) ./src/main/fc/rc_modes.c 
%% (optimised) ./src/main/flight/position.c 
%% (optimised) ./src/main/flight/failsafe.c 
%% (optimised) ./src/main/flight/gps_rescue.c 
%% (optimised) ./src/main/fc/gps_lap_timer.c 
%% (speed optimised) ./src/main/flight/dyn_notch_filter.c 
%% (speed optimised) ./src/main/flight/mixer.c 
%% (size optimised) ./src/main/flight/mixer_init.c 
%% (optimised) ./src/main/flight/mixer_tricopter.c 
%% (speed optimised) ./src/main/flight/pid.c 
%% (size optimised) ./src/main/flight/pid_init.c 
%% (speed optimised) ./src/main/flight/rpm_filter.c 
%% (speed optimised) ./src/main/flight/imu.c 
%% (optimised) ./src/main/flight/servos.c 
%% (size optimised) ./src/main/io/serial_4way.c 
%% (optimised) ./src/main/flight/servos_tricopter.c 
%% (size optimised) ./src/main/io/serial_4way_avrootloader.c 
%% (size optimised) ./src/main/io/serial_4way_stk500v2.c 
%% (speed optimised) ./src/main/rx/ibus.c 
%% (optimised) ./src/main/rx/jetiexbus.c 
%% (optimised) ./src/main/rx/msp.c 
%% (speed optimised) ./src/main/rx/frsky_crc.c 
%% (speed optimised) ./src/main/rx/rc_stats.c 
%% (speed optimised) ./src/main/rx/rx.c 
%% (speed optimised) ./src/main/rx/rx_spi.c 
%% (size optimised) ./src/main/rx/rx_bind.c 
%% (optimised) ./src/main/rx/pwm.c 
%% (optimised) ./src/main/rx/rx_spi_common.c 
%% (speed optimised) ./src/main/rx/crsf.c 
%% (optimised) ./src/main/rx/ghst.c 
%% (speed optimised) ./src/main/rx/sbus.c 
%% (speed optimised) ./src/main/rx/sbus_channels.c 
%% (speed optimised) ./src/main/rx/srxl2.c 
%% (optimised) ./src/main/io/spektrum_rssi.c 
%% (size optimised) ./src/main/io/spektrum_vtx_control.c 
%% (speed optimised) ./src/main/rx/spektrum.c 
%% (speed optimised) ./src/main/rx/sumd.c 
%% (optimised) ./src/main/rx/sumh.c 
%% (speed optimised) ./src/main/rx/xbus.c 
%% (speed optimised) ./src/main/rx/fport.c 
%% (optimised) ./src/main/rx/msp_override.c 
%% (size optimised) ./src/main/sensors/acceleration_init.c 
%% (speed optimised) ./src/main/sensors/acceleration.c 
%% (optimised) ./src/main/sensors/compass.c 
%% (speed optimised) ./src/main/sensors/gyro.c 
%% (size optimised) ./src/main/sensors/gyro_init.c 
%% (speed optimised) ./src/main/sensors/boardalignment.c 
%% (optimised) ./src/main/sensors/initialisation.c 
%% (optimised) ./src/main/blackbox/blackbox.c 
%% (optimised) ./src/main/blackbox/blackbox_io.c 
%% (optimised) ./src/main/blackbox/blackbox_encoding.c 
%% (size optimised) ./src/main/cms/cms.c 
%% (size optimised) ./src/main/cms/cms_menu_failsafe.c 
%% (size optimised) ./src/main/cms/cms_menu_blackbox.c 
%% (size optimised) ./src/main/cms/cms_menu_firmware.c 
%% (size optimised) ./src/main/cms/cms_menu_gps_rescue.c 
%% (size optimised) ./src/main/cms/cms_menu_gps_lap_timer.c 
%% (size optimised) ./src/main/cms/cms_menu_imu.c 
%% (size optimised) ./src/main/cms/cms_menu_ledstrip.c 
%% (size optimised) ./src/main/cms/cms_menu_main.c 
%% (size optimised) ./src/main/cms/cms_menu_misc.c 
%% (size optimised) ./src/main/cms/cms_menu_osd.c 
%% (size optimised) ./src/main/cms/cms_menu_saveexit.c 
%% (size optimised) ./src/main/cms/cms_menu_power.c 
%% (size optimised) ./src/main/cms/cms_menu_vtx_common.c 
%% (size optimised) ./src/main/cms/cms_menu_vtx_rtc6705.c 
%% (size optimised) ./src/main/cms/cms_menu_vtx_smartaudio.c 
%% (size optimised) ./src/main/cms/cms_menu_vtx_tramp.c 
%% (size optimised) ./src/main/cms/cms_menu_persistent_stats.c 
%% (size optimised) ./src/main/cms/cms_menu_quick.c 
%% (size optimised) ./src/main/drivers/light_ws2811strip.c 
%% (size optimised) ./src/main/cms/cms_menu_rpm_limit.c 
%% (optimised) ./src/main/drivers/rangefinder/rangefinder_hcsr04.c 
%% (optimised) ./src/main/drivers/rangefinder/rangefinder_lidartf.c 
%% (size optimised) ./src/main/drivers/vtx_common.c 
%% (optimised) ./src/main/drivers/vtx_table.c 
%% (size optimised) ./src/main/io/dashboard.c 
%% (optimised) ./src/main/io/displayport_frsky_osd.c 
%% (optimised) ./src/main/io/displayport_max7456.c 
%% (optimised) ./src/main/io/displayport_msp.c 
%% (optimised) ./src/main/io/displayport_srxl.c 
%% (optimised) ./src/main/io/displayport_crsf.c 
%% (optimised) ./src/main/io/displayport_hott.c 
%% (optimised) ./src/main/io/rcdevice_cam.c 
%% (optimised) ./src/main/io/frsky_osd.c 
%% (optimised) ./src/main/io/rcdevice.c 
%% (optimised) ./src/main/io/gps.c 
%% (optimised) ./src/main/io/ledstrip.c 
%% (optimised) ./src/main/io/pidaudio.c 
%% (size optimised) ./src/main/osd/osd.c 
%% (size optimised) ./src/main/osd/osd_warnings.c 
%% (size optimised) ./src/main/osd/osd_elements.c 
%% (optimised) ./src/main/sensors/barometer.c 
%% (optimised) ./src/main/sensors/rangefinder.c 
%% (optimised) ./src/main/telemetry/telemetry.c 
%% (optimised) ./src/main/telemetry/frsky_hub.c 
%% (optimised) ./src/main/telemetry/jetiexbus.c 
%% (optimised) ./src/main/telemetry/hott.c 
%% (optimised) ./src/main/telemetry/smartport.c 
%% (optimised) ./src/main/telemetry/mavlink.c 
%% (optimised) ./src/main/telemetry/ltm.c 
%% (optimised) ./src/main/telemetry/msp_shared.c 
%% (optimised) ./src/main/telemetry/ibus.c 
%% (optimised) ./src/main/telemetry/ibus_shared.c 
%% (optimised) ./src/main/sensors/esc_sensor.c 
%% (size optimised) ./src/main/io/vtx.c 
%% (size optimised) ./src/main/io/vtx_rtc6705.c 
%% (size optimised) ./src/main/io/vtx_smartaudio.c 
%% (size optimised) ./src/main/io/vtx_tramp.c 
%% (size optimised) ./src/main/io/vtx_control.c 
%% (size optimised) ./src/main/io/vtx_msp.c 
%% (size optimised) ./src/main/cms/cms_menu_vtx_msp.c 
%% (size optimised) lib/main/google/olc/olc.c 
Linking SITL 
lto-wrapper: warning: using serial compilation of 4 LTRANS jobs
   text	   data	    bss	    dec	    hex	filename
 281037	  16284	  73792	 371113	  5a9a9	./obj/main/betaflight_SITL.elf
Creating HEX ./obj/betaflight_4.5.0_SITL.hex 
make[1]: Leaving directory '/home/alessandro/Desktop/CFRRS2_Apr_24/gym-pybullet-drones/betaflight_sitl/bf0'
make -j ./obj/betaflight_4.5.0_SITL.hex
make[1]: Entering directory '/home/alessandro/Desktop/CFRRS2_Apr_24/gym-pybullet-drones/betaflight_sitl/bf1'
rm -f ./obj/main/SITL/.efhash_*
EF HASH -> ./obj/main/SITL/.efhash_d41d8cd98f00b204e9800998ecf8427e
%% (optimised) ./src/main/drivers/accgyro/accgyro_virtual.c 
%% (optimised) lib/main/dyad/dyad.c 
%% (optimised) ./src/main/drivers/barometer/barometer_virtual.c 
%% (optimised) ./src/main/drivers/compass/compass_virtual.c 
%% (size optimised) ./src/main/drivers/serial_tcp.c 
%% (optimised) ./src/main/build/build_config.c 
%% (optimised) ./src/main/build/debug.c 
%% (optimised) ./src/main/build/debug_pin.c 
%% (optimised) ./src/main/build/version.c 
%% (optimised) ./src/main/target/SITL/sitl.c 
%% (optimised) ./src/main/target/SITL/udplink.c 
%% (size optimised) ./src/main/main.c 
%% (optimised) ./src/main/pg/adc.c 
%% (optimised) ./src/main/pg/beeper.c 
%% (optimised) ./src/main/pg/beeper_dev.c 
%% (optimised) ./src/main/pg/board.c 
%% (optimised) ./src/main/pg/bus_i2c.c 
%% (optimised) ./src/main/pg/bus_quadspi.c 
%% (optimised) ./src/main/pg/bus_spi.c 
%% (optimised) ./src/main/pg/dashboard.c 
%% (optimised) ./src/main/pg/displayport_profiles.c 
%% (optimised) ./src/main/pg/dyn_notch.c 
%% (optimised) ./src/main/pg/flash.c 
%% (optimised) ./src/main/pg/gps.c 
%% (optimised) ./src/main/pg/gps_lap_timer.c 
%% (optimised) ./src/main/pg/gps_rescue.c 
%% (optimised) ./src/main/pg/gyrodev.c 
%% (optimised) ./src/main/pg/max7456.c 
%% (optimised) ./src/main/pg/mco.c 
%% (optimised) ./src/main/pg/msp.c 
%% (optimised) ./src/main/pg/motor.c 
%% (optimised) ./src/main/pg/pg.c 
%% (optimised) ./src/main/pg/piniobox.c 
%% (optimised) ./src/main/pg/pinio.c 
%% (optimised) ./src/main/pg/pin_pull_up_down.c 
%% (optimised) ./src/main/pg/rcdevice.c 
%% (optimised) ./src/main/pg/rpm_filter.c 
%% (optimised) ./src/main/pg/rx.c 
%% (optimised) ./src/main/pg/rx_pwm.c 
%% (optimised) ./src/main/pg/rx_spi.c 
%% (optimised) ./src/main/pg/rx_spi_cc2500.c 
%% (optimised) ./src/main/pg/rx_spi_expresslrs.c 
%% (optimised) ./src/main/pg/scheduler.c 
%% (optimised) ./src/main/pg/sdcard.c 
%% (optimised) ./src/main/pg/sdio.c 
%% (optimised) ./src/main/pg/serial_uart.c 
%% (optimised) ./src/main/pg/stats.c 
%% (optimised) ./src/main/pg/timerio.c 
%% (optimised) ./src/main/pg/timerup.c 
%% (optimised) ./src/main/pg/usb.c 
%% (optimised) ./src/main/pg/vcd.c 
%% (optimised) ./src/main/pg/vtx_io.c 
%% (optimised) ./src/main/pg/vtx_table.c 
%% (optimised) ./src/main/common/bitarray.c 
%% (optimised) ./src/main/common/colorconversion.c 
%% (optimised) ./src/main/common/crc.c 
%% (speed optimised) ./src/main/common/encoding.c 
%% (optimised) ./src/main/common/explog_approx.c 
%% (speed optimised) ./src/main/common/filter.c 
%% (optimised) ./src/main/common/gps_conversion.c 
%% (optimised) ./src/main/common/huffman.c 
%% (optimised) ./src/main/common/huffman_table.c 
%% (speed optimised) ./src/main/common/maths.c 
%% (optimised) ./src/main/common/printf.c 
%% (optimised) ./src/main/common/printf_serial.c 
%% (speed optimised) ./src/main/common/sdft.c 
%% (optimised) ./src/main/common/sensor_alignment.c 
%% (speed optimised) ./src/main/common/stopwatch.c 
%% (optimised) ./src/main/common/streambuf.c 
%% (optimised) ./src/main/common/string_light.c 
%% (optimised) ./src/main/common/strtol.c 
%% (optimised) ./src/main/common/time.c 
%% (speed optimised) ./src/main/common/typeconversion.c 
%% (optimised) ./src/main/common/uvarint.c 
%% (optimised) ./src/main/config/config.c 
%% (size optimised) ./src/main/config/config_eeprom.c 
%% (size optimised) ./src/main/config/config_streamer.c 
%% (size optimised) ./src/main/config/feature.c 
%% (size optimised) ./src/main/config/simplified_tuning.c 
%% (size optimised) ./src/main/cli/cli.c 
%% (size optimised) ./src/main/cli/settings.c 
%% (optimised) ./src/main/drivers/dshot.c 
%% (optimised) ./src/main/drivers/dshot_dpwm.c 
%% (optimised) ./src/main/drivers/dshot_command.c 
%% (speed optimised) ./src/main/drivers/buf_writer.c 
%% (speed optimised) ./src/main/drivers/bus.c 
%% (optimised) ./src/main/drivers/bus_i2c_busdev.c 
%% (optimised) ./src/main/drivers/bus_i2c_utils.c 
%% (optimised) ./src/main/drivers/bus_i2c_soft.c 
%% (optimised) ./src/main/drivers/bus_octospi.c 
%% (speed optimised) ./src/main/drivers/bus_quadspi.c 
%% (optimised) ./src/main/drivers/buttons.c 
%% (optimised) ./src/main/drivers/display.c 
%% (optimised) ./src/main/drivers/display_canvas.c 
%% (optimised) ./src/main/drivers/dma_common.c 
%% (speed optimised) ./src/main/drivers/io.c 
%% (optimised) ./src/main/drivers/light_led.c 
%% (optimised) ./src/main/drivers/mco.c 
%% (optimised) ./src/main/drivers/motor.c 
%% (optimised) ./src/main/drivers/pinio.c 
%% (optimised) ./src/main/drivers/pin_pull_up_down.c 
%% (optimised) ./src/main/drivers/resource.c 
%% (speed optimised) ./src/main/drivers/serial.c 
%% (optimised) ./src/main/drivers/sound_beeper.c 
%% (optimised) ./src/main/drivers/stack_check.c 
%% (optimised) ./src/main/drivers/timer_common.c 
%% (optimised) ./src/main/drivers/transponder_ir_arcitimer.c 
%% (optimised) ./src/main/drivers/transponder_ir_ilap.c 
%% (optimised) ./src/main/drivers/transponder_ir_erlt.c 
%% (size optimised) ./src/main/fc/board_info.c 
%% (optimised) ./src/main/fc/hardfaults.c 
%% (optimised) ./src/main/fc/dispatch.c 
%% (speed optimised) ./src/main/fc/tasks.c 
%% (speed optimised) ./src/main/fc/runtime_config.c 
%% (optimised) ./src/main/fc/stats.c 
%% (optimised) ./src/main/io/beeper.c 
%% (optimised) ./src/main/io/piniobox.c 
%% (size optimised) ./src/main/io/serial.c 
%% (optimised) ./src/main/io/smartaudio_protocol.c 
%% (optimised) ./src/main/io/statusindicator.c 
%% (optimised) ./src/main/io/tramp_protocol.c 
%% (size optimised) ./src/main/io/usb_cdc_hid.c 
%% (size optimised) ./src/main/io/transponder_ir.c 
%% (optimised) ./src/main/io/usb_msc.c 
%% (optimised) ./src/main/msp/msp.c 
%% (optimised) ./src/main/msp/msp_box.c 
%% (size optimised) ./src/main/msp/msp_serial.c 
%% (optimised) ./src/main/msp/msp_build_info.c 
%% (speed optimised) ./src/main/scheduler/scheduler.c 
%% (optimised) ./src/main/sensors/adcinternal.c 
%% (optimised) ./src/main/sensors/battery.c 
%% (optimised) ./src/main/sensors/current.c 
%% (optimised) ./src/main/sensors/voltage.c 
%% (optimised) ./src/main/target/config_helper.c 
%% (size optimised) ./src/main/fc/init.c 
%% (optimised) ./src/main/drivers/accgyro/gyro_sync.c 
%% (optimised) ./src/main/fc/controlrate_profile.c 
%% (optimised) ./src/main/drivers/rx/rx_spi.c 
%% (optimised) ./src/main/drivers/rx/rx_pwm.c 
%% (optimised) ./src/main/drivers/serial_softserial.c 
%% (speed optimised) ./src/main/fc/core.c 
%% (speed optimised) ./src/main/fc/rc.c 
%% (speed optimised) ./src/main/fc/rc_controls.c 
%% (optimised) ./src/main/fc/rc_adjustments.c 
%% (optimised) ./src/main/fc/rc_modes.c 
%% (optimised) ./src/main/flight/position.c 
%% (optimised) ./src/main/flight/gps_rescue.c 
%% (optimised) ./src/main/flight/failsafe.c 
%% (optimised) ./src/main/fc/gps_lap_timer.c 
%% (speed optimised) ./src/main/flight/dyn_notch_filter.c 
%% (speed optimised) ./src/main/flight/imu.c 
%% (speed optimised) ./src/main/flight/mixer.c 
%% (size optimised) ./src/main/flight/mixer_init.c 
%% (optimised) ./src/main/flight/mixer_tricopter.c 
%% (speed optimised) ./src/main/flight/pid.c 
%% (size optimised) ./src/main/flight/pid_init.c 
%% (optimised) ./src/main/flight/servos.c 
%% (speed optimised) ./src/main/flight/rpm_filter.c 
%% (optimised) ./src/main/flight/servos_tricopter.c 
%% (size optimised) ./src/main/io/serial_4way.c 
%% (size optimised) ./src/main/io/serial_4way_avrootloader.c 
%% (size optimised) ./src/main/io/serial_4way_stk500v2.c 
%% (speed optimised) ./src/main/rx/ibus.c 
%% (optimised) ./src/main/rx/jetiexbus.c 
%% (optimised) ./src/main/rx/msp.c 
%% (optimised) ./src/main/rx/pwm.c 
%% (speed optimised) ./src/main/rx/frsky_crc.c 
%% (speed optimised) ./src/main/rx/rc_stats.c 
%% (size optimised) ./src/main/rx/rx_bind.c 
%% (speed optimised) ./src/main/rx/rx.c 
%% (speed optimised) ./src/main/rx/rx_spi.c 
%% (optimised) ./src/main/rx/rx_spi_common.c 
%% (optimised) ./src/main/rx/ghst.c 
%% (speed optimised) ./src/main/rx/crsf.c 
%% (speed optimised) ./src/main/rx/sbus.c 
%% (speed optimised) ./src/main/rx/sbus_channels.c 
%% (speed optimised) ./src/main/rx/spektrum.c 
%% (speed optimised) ./src/main/rx/srxl2.c 
%% (size optimised) ./src/main/io/spektrum_vtx_control.c 
%% (optimised) ./src/main/io/spektrum_rssi.c 
%% (speed optimised) ./src/main/rx/sumd.c 
%% (optimised) ./src/main/rx/sumh.c 
%% (speed optimised) ./src/main/rx/xbus.c 
%% (speed optimised) ./src/main/rx/fport.c 
%% (optimised) ./src/main/rx/msp_override.c 
%% (speed optimised) ./src/main/sensors/acceleration.c 
%% (size optimised) ./src/main/sensors/acceleration_init.c 
%% (speed optimised) ./src/main/sensors/boardalignment.c 
%% (optimised) ./src/main/sensors/compass.c 
%% (speed optimised) ./src/main/sensors/gyro.c 
%% (optimised) ./src/main/sensors/initialisation.c 
%% (size optimised) ./src/main/sensors/gyro_init.c 
%% (optimised) ./src/main/blackbox/blackbox.c 
%% (optimised) ./src/main/blackbox/blackbox_encoding.c 
%% (optimised) ./src/main/blackbox/blackbox_io.c 
%% (size optimised) ./src/main/cms/cms.c 
%% (size optimised) ./src/main/cms/cms_menu_blackbox.c 
%% (size optimised) ./src/main/cms/cms_menu_failsafe.c 
%% (size optimised) ./src/main/cms/cms_menu_firmware.c 
%% (size optimised) ./src/main/cms/cms_menu_gps_rescue.c 
%% (size optimised) ./src/main/cms/cms_menu_gps_lap_timer.c 
%% (size optimised) ./src/main/cms/cms_menu_imu.c 
%% (size optimised) ./src/main/cms/cms_menu_ledstrip.c 
%% (size optimised) ./src/main/cms/cms_menu_main.c 
%% (size optimised) ./src/main/cms/cms_menu_misc.c 
%% (size optimised) ./src/main/cms/cms_menu_osd.c 
%% (size optimised) ./src/main/cms/cms_menu_power.c 
%% (size optimised) ./src/main/cms/cms_menu_saveexit.c 
%% (size optimised) ./src/main/cms/cms_menu_vtx_common.c 
%% (size optimised) ./src/main/cms/cms_menu_vtx_rtc6705.c 
%% (size optimised) ./src/main/cms/cms_menu_vtx_smartaudio.c 
%% (size optimised) ./src/main/cms/cms_menu_vtx_tramp.c 
%% (size optimised) ./src/main/cms/cms_menu_persistent_stats.c 
%% (size optimised) ./src/main/cms/cms_menu_rpm_limit.c 
%% (size optimised) ./src/main/cms/cms_menu_quick.c 
%% (size optimised) ./src/main/drivers/light_ws2811strip.c 
%% (optimised) ./src/main/drivers/rangefinder/rangefinder_hcsr04.c 
%% (optimised) ./src/main/drivers/rangefinder/rangefinder_lidartf.c 
%% (size optimised) ./src/main/drivers/vtx_common.c 
%% (optimised) ./src/main/drivers/vtx_table.c 
%% (size optimised) ./src/main/io/dashboard.c 
%% (optimised) ./src/main/io/displayport_frsky_osd.c 
%% (optimised) ./src/main/io/displayport_max7456.c 
%% (optimised) ./src/main/io/displayport_msp.c 
%% (optimised) ./src/main/io/displayport_srxl.c 
%% (optimised) ./src/main/io/displayport_crsf.c 
%% (optimised) ./src/main/io/displayport_hott.c 
%% (optimised) ./src/main/io/frsky_osd.c 
%% (optimised) ./src/main/io/rcdevice_cam.c 
%% (optimised) ./src/main/io/rcdevice.c 
%% (optimised) ./src/main/io/gps.c 
%% (optimised) ./src/main/io/ledstrip.c 
%% (size optimised) ./src/main/osd/osd.c 
%% (optimised) ./src/main/io/pidaudio.c 
%% (size optimised) ./src/main/osd/osd_elements.c 
%% (size optimised) ./src/main/osd/osd_warnings.c 
%% (optimised) ./src/main/sensors/barometer.c 
%% (optimised) ./src/main/sensors/rangefinder.c 
%% (optimised) ./src/main/telemetry/telemetry.c 
%% (optimised) ./src/main/telemetry/frsky_hub.c 
%% (optimised) ./src/main/telemetry/hott.c 
%% (optimised) ./src/main/telemetry/jetiexbus.c 
%% (optimised) ./src/main/telemetry/smartport.c 
%% (optimised) ./src/main/telemetry/ltm.c 
%% (optimised) ./src/main/telemetry/mavlink.c 
%% (optimised) ./src/main/telemetry/msp_shared.c 
%% (optimised) ./src/main/telemetry/ibus.c 
%% (optimised) ./src/main/telemetry/ibus_shared.c 
%% (optimised) ./src/main/sensors/esc_sensor.c 
%% (size optimised) ./src/main/io/vtx.c 
%% (size optimised) ./src/main/io/vtx_rtc6705.c 
%% (size optimised) ./src/main/io/vtx_smartaudio.c 
%% (size optimised) ./src/main/io/vtx_tramp.c 
%% (size optimised) ./src/main/io/vtx_control.c 
%% (size optimised) ./src/main/io/vtx_msp.c 
%% (size optimised) ./src/main/cms/cms_menu_vtx_msp.c 
%% (size optimised) lib/main/google/olc/olc.c 
Linking SITL 
lto-wrapper: warning: using serial compilation of 4 LTRANS jobs
   text	   data	    bss	    dec	    hex	filename
 281037	  16284	  73792	 371113	  5a9a9	./obj/main/betaflight_SITL.elf
Creating HEX ./obj/betaflight_4.5.0_SITL.hex 
make[1]: Leaving directory '/home/alessandro/Desktop/CFRRS2_Apr_24/gym-pybullet-drones/betaflight_sitl/bf1'

Nello stesso terminale, seguendo sempre la parte commentata di beta.py, runno:

$ cd betaflight_sitl/bf0/
$ ./obj/main/betaflight_SITL.elf

apro un nuovo terminale in gym-pybullet-drones e runno:

$ conda activate drones
$ cd betaflight_sitl/bf1/
$ ./obj/main/betaflight_SITL.elf

gli output di questi ultimi due comandi sono molto lunghi, riporto la fine di entrambi:

-primo

...
[FLASH_Lock] saved 'eeprom.bin'
bind port 5761 for UART1
debugInit
unusedPinsInit

-secondo

...
[FLASH_Lock] saved 'eeprom.bin'
bind port 5761 for UART1 failed!!
debugInit
unusedPinsInit

Apro un nuovo terminale in gym-pybullet-drones e runno:

$ conda activate drones
$ cd gym_pybullet_drones/examples/
$ python3 beta.py

Si apre la finestra grafica di bullet che muore subito e si aprono altri due terminali.
Nel terminale in cui ho dato i comandi il messaggio di errore è il seguente:

pybullet build time: Sep  8 2023 16:57:18
[INFO] BaseAviary.__init__() loaded parameters from the drone's .urdf:
[INFO] m 0.830000, L 0.109000,
[INFO] ixx 0.003113, iyy 0.003113, izz 0.003113,
[INFO] kf 0.000000, km 0.000000,
[INFO] t2w 4.170000, max_speed_kmh 200.000000,
[INFO] gnd_eff_coeff 11.368590, prop_radius 0.127000,
[INFO] drag_xy_coeff 0.000001, drag_z_coeff 0.000001,
[INFO] dw_coeff_1 2267.180000, dw_coeff_2 0.160000, dw_coeff_3 -0.110000
startThreads creating 1 threads.
starting thread 0
started thread 0 
argc=2
argv[0] = --unused
argv[1] = --start_demo_name=Physics Server
ExampleBrowserThreadFunc started
X11 functions dynamically loaded using dlopen/dlsym OK!
X11 functions dynamically loaded using dlopen/dlsym OK!
Creating context
Created GL 3.3 context
Direct GLX rendering context obtained
Making context current
GL_VENDOR=Intel
GL_RENDERER=Mesa Intel(R) UHD Graphics (CML GT2)
GL_VERSION=4.6 (Core Profile) Mesa 23.0.4-0ubuntu1~22.04.1
GL_SHADING_LANGUAGE_VERSION=4.60
pthread_getconcurrency()=0
Version = 4.6 (Core Profile) Mesa 23.0.4-0ubuntu1~22.04.1
Vendor = Intel
Renderer = Mesa Intel(R) UHD Graphics (CML GT2)
b3Printf: Selected demo: Physics Server
startThreads creating 1 threads.
starting thread 0
started thread 0 
MotionThreadFunc thread started
viewMatrix (-0.8660253882408142, -0.2499999701976776, 0.4330126941204071, 0.0, 0.0, 0.8660253286361694, 0.4999999701976776, 0.0, -0.4999999701976776, 0.4330126643180847, -0.75, 0.0, -0.0, 5.960464477539063e-08, -2.999999761581421, 1.0)
projectionMatrix (1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0000200271606445, -1.0, 0.0, 0.0, -0.02000020071864128, 0.0)
/home/alessandro/anaconda3/envs/drones/lib/python3.10/site-packages/gymnasium/spaces/box.py:130: UserWarning: WARN: Box bound precision lowered by casting to float32
  gym.logger.warn(f"Box bound precision lowered by casting to {self.dtype}")
ven = Intel
Workaround for some crash in the Intel OpenGL driver on Linux/Ubuntu
ven = Intel
Workaround for some crash in the Intel OpenGL driver on Linux/Ubuntu
Traceback (most recent call last):
  File "/home/alessandro/Desktop/CFRRS2_Apr_24/gym-pybullet-drones/gym_pybullet_drones/examples/beta.py", line 182, in <module>
    run(**vars(ARGS))
  File "/home/alessandro/Desktop/CFRRS2_Apr_24/gym-pybullet-drones/gym_pybullet_drones/examples/beta.py", line 105, in run
    with open("../assets/beta.csv", mode='r') as csv_file:
FileNotFoundError: [Errno 2] No such file or directory: '../assets/beta.csv'
numActiveThreads = 0
stopping threads
Thread with taskId 0 exiting
Thread TERMINATED
destroy semaphore
semaphore destroyed
destroy main semaphore
main semaphore destroyed
finished
numActiveThreads = 0
btShutDownExampleBrowser stopping threads
Thread with taskId 0 exiting
Thread TERMINATED
destroy semaphore
semaphore destroyed
destroy main semaphore
main semaphore destroyed

I due terminali che si aprono dop hanno lo stesso output che riporta:

[SITL] The SITL will output to IP 127.0.0.1:9002 (Gazebo) and 127.0.0.1:9001 (RealFlightBridge)
[system]Init...
[SITL] init PwmOut UDP link to gazebo 127.0.0.1:9002...0
[SITL] init PwmOut UDP link to RF9 127.0.0.1:9001...0
[SITL] start UDP server @9003...0
[SITL] start UDP server for RC input @9004...0
[FLASH_Unlock] loaded 'eeprom.bin', size = 32768 / 32768
[timer]Init...
bind port 5761 for UART1 failed!!
debugInit
unusedPinsInit

Con la versione nuova ste teste di minchia hanno cambiato il nome del file beta.csv nella directory '../assets/' in beta-traj.csv senza però cambiarlo quando lo evocano in beta.py porcoddio. Cambio il nome del file beta-traj.csv in beta.csv, anche se comunque prevedo errori. I terminali che ho usato alle righe 681 e 686 sono ancora aperti e runnanti. Rirunno beta.py nel terminale dove l'ho già runnato:

$ python3 beta.py

stesso identico errore, unica differenza: gli errori che ho incollato alle linee 763 e 764 di questo file adesso riportano

File "/home/alessandro/Desktop/CFRRS2_Apr_24/gym-pybullet-drones/gym_pybullet_drones/examples/beta.py", line 91, in run
    with open("../assets/beta-traj.csv", mode='r') as csv_file:
FileNotFoundError: [Errno 2] No such file or directory: '../assets/beta-traj.csv'

Mi stanno chiaramente prendendo per il culo. Riporto il nome del file beta.csv a quello originale (beta-traj.csv) e cambio la riga 105 di beta.py:

da
with open("../assets/beta.csv", mode='r') as csv_file:

a
with open("../assets/beta-traj.csv", mode='r') as csv_file:

rirunno lo script aspettando errori dovuti alla porta 5761. I due terminali si aprono comunque ma la simulazione adesso runna per tutto il tempo prestabilito, vengono caricati due droni che però stanno fermi, cosa che riflettono anche i plot post simulazione. I due terminali hanno lo stesso output che riporta:

[SITL] The SITL will output to IP 127.0.0.1:9012 (Gazebo) and 127.0.0.1:9011 (RealFlightBridge)
[system]Init...
[SITL] init PwmOut UDP link to gazebo 127.0.0.1:9012...0
[SITL] init PwmOut UDP link to RF9 127.0.0.1:9011...0
[SITL] start UDP server @9013...0
[SITL] start UDP server for RC input @9014...0
[FLASH_Unlock] loaded 'eeprom.bin', size = 32768 / 32768
[timer]Init...
bind port 5761 for UART1 failed!!
debugInit
unusedPinsInit
[SITL] new fdm 144 t:0.000000 from 0.0.0.0:0
[SITL] new rc 40: t:0.000000 AETR: 1500 1500 1000 1500 AUX1-4: 1000 1000 1000 1000

killo tutti i processi aperti relativi ai droni (Ctrl+C nei terminali delle righe 681 e 686)
analizzando lo script clone_bfs.sh in '.../assets' (che configura betaflight automaticamente) con la parte commentata di beta.py della vecchia versione (che faceva configuarare betaflight manualmente), sembra che la nuova versione si fermi allo step 2 ($ make TARGET=SITL). Proverei a ricaricare il file .txt da betaflight configurator nei preset della porta 5761, ma al momento è comunque caricato quello della versione vecchia (che è lo stesso della nuova) e la roba non gira.
Col fatto che clone_bfs.sh configura betaflight in ogni cartella relativa ad ogni singolo drone (e.g. se ho selezionato 2 come numero max di droni possibili, dentro "/gym-pybullet-drones/betaflight_sitl" mi trovo due cartelle, bf0 e bf1, ognuna relativa a un drone) proverei a ripetere gli step 3 e 4 del vecchio beta.py per entrambi i droni e vedere che succede.  
Primo tentativo: apro solo un drone--> solo un terminale in bf0, faccio la magia dal configurator e provo a runnare beta.py. Apro terminale in gym-pybulet-drones e runno:

$ conda activate drones
$ cd betaflight_sitl/bf0/

apro betaflight, seleziono "manual selection" per la porta e nel riquadro che appare a sx del menu a tendina scrivo tcp://localhost:5761. Connetto. Nel menu a sx seleziono Presets-->load backup, vado in ".../assets" e seleziono "beta-presets-bak.txt".
Nel terminale, terminato il processo precedente, runno:

$ ./obj/main/betaflight_SITL.elf

Apro un altro terminale in gym-pybullet-drones, attivo l'ambiante drones e runno:

$ cd gym_pybullet_drones/examples/
$ python beta.py

non è cambiato nulla. Provo

$ python beta.py --num_drones 1

così si apre comunque il terminale aggiuntivo che dice la roba qui sotto, però un singolo drone fa quello che faceva nella vecchia versione!

[SITL] The SITL will output to IP 127.0.0.1:9002 (Gazebo) and 127.0.0.1:9001 (RealFlightBridge)
[system]Init...
[SITL] init PwmOut UDP link to gazebo 127.0.0.1:9002...0
[SITL] init PwmOut UDP link to RF9 127.0.0.1:9001...0
[SITL] start UDP server @9003...0
[SITL] start UDP server for RC input @9004...0
[FLASH_Unlock] loaded 'eeprom.bin', size = 32768 / 32768
[timer]Init...
Initialized motor count 4
bind port 5761 for UART1 failed!!
debugInit
unusedPinsInit
[SITL] new fdm 144 t:0.000000 from 0.0.0.0:0
[SITL] new rc 40: t:0.000000 AETR: 1500 1500 1000 1500 AUX1-4: 1000 1000 1000 1000

chiudo il processo che girava dentro "gym-pybullet-drones/betaflight_sitl/bf0". Apro nuovo terminale in gym-pybullet-drones e runno:

$ cd betaflight_sitl/bf1/
$ ./obj/main/betaflight_SITL.elf

faccio quanto fatto a riga 840. Se lo fai, dopo che runni ./obj/main/betaflight_SITL.elf, tra gli output c'è una riga che riporta "initialized motor count 4", che mi fa capire che beta.py può girare; se non lo fai questa riga non c'è. Provo a runnare ./obj/main/betaflight_SITL.elf in bf0 e bf1 e poi beta.py.
bf0 ha le due righe importanti:

Initialized motor count 4
bind port 5761 for UART1

bf1 ha

Initialized motor count 4
una marea di altre righe con codici tipo byte
bind port 5761 for UART1 failed!!
 
se runni prima da bf1 e poi bf0 i risultati si invertono, ma in bf0 non c'è la marea di righe di cui sopra. Provo a runnare beta.py: il drone 1 fa il giretto che deve fare, lo 0 giustamente no. I due terminali aggiuntivi escono fuori sempre.
Tocca capire come bindare ogni drone su un port diverso.











