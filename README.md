# STM8TILink
The TI grey link clone based on arduino code and half duplex code for stm8s (credits after this line). WIP

# credit 
[DSchndr](https://github.com/DSchndr/serial2ti83) fork of [jw0k repo](https://github.com/jw0k/serial2ti83).

# PCB (at OSHpark).
[rev1 PCB](https://oshpark.com/shared_projects/LCwtGfEf).

# Extra
Due to the physical limitation of STM8S001J3 available pin. I was forced to use the single wire bi-directional UART. which it means that I share TX and RX on same pin (based on [This guy work](http://ficara.altervista.org/?p=4047&doing_wp_cron=1633419018.3038520812988281250000)). The problem starts when host send data to CH330N, since the TX is  connected to schottky barrier diode on cathod side, anode side is connected to STM8 (with 4.7k pull up) and RX pin of CH330N. The loopback "echo" occured everytime the host send command to STM8. The CH330N doesn't ignore the RX line when TX is transmitting. SO the host confused that the RX data is valid (which is not!). In order to solve this problem. I redesigned the PCB with included the SN74LVC1G3157DBVR, SPDT analog switch. I use this IC for bus isolation. CH330N RX will be isolated when MCU isn't send any data to host, it will left unconnected all the time to prevent the loopback echo and confuse host device.
