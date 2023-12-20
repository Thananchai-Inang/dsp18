https://www.cybertice.com/article/732/สอนใช้งาน-esp32-l298n-โมดูลขับมอเตอร์-ควบคุมทิศทางการหมุนของมอเตอร์

L298N Motor Drive Module 

โมดูล L298N ขับมอเตอร์ได้ 2 ตัวแบบแยกอิสระ สามารถควบคุมความเร็วมอเตอร์ได้ ใช้ไฟ 5 โวลต์ เลี้ยงบอร์ดได้สามารถรับไฟเข้า 7-35 โวลต์ขับมอเตอร์ได้ มีวงจรเรกูเลตในตัว ขับกระแสสูงสุดได้ 2A

Motor Drive Module L298N

This module is integrated with a built-in 5V power.When the drive voltage is 7V-35V, it can enable the onboard 5V logic power supply;afterthe power supply,don't input voltage in the interface+5V power supply,but you can lead the 5V for external use. 
when ENA enable IN1 IN2 control OUT1 OUT2 
when ENB enable IN3 IN4 control OUT3 OUT4

Applied cases: 
1 Driving stepper motor 
The connection of driving a common 4 line 2 phase electric motor is shown in below figure 
after enable ENA ENB 
Input the following driving timing from IN1-IN4，then the speed and direction of the stepper 
motor can be controlled

2 Driving DC motor 
Because the module is drived by double H bridge,it can drive two motors simultaneously. 
The connecting method is shown inbelow figure 
after enable ENA ENB 
You can imput the speed and direction of PWM signal drive motor 1 from IN1 IN2 
You can imput the speed and direction of PWM signal drive motor 2 from IN1 IN2 
The signal is shown in the figure 


คุณสมบัติโมดูลขับมอเตอร์ L298N

Double H bridge drive
Chip: L298N (ST NEW)
Logical voltage: 5V
Drive voltage: 7V-35V
Logical current: 0mA-36mA
Drive current: 2A(MAX single bridge)
Max power: 25W
Size:43 x 43 x 26mm
Net weight: 26g
Package weight:36g
 

วิธีการต่ออุปกรณ์ สอนใช้งาน Arduino L298N Motor Drive Module

1.เชื่อมต่ออุปกรณ์ตามด้านล่าง
Arduino UNO R3 -> L298N Motor Drive Module
            • 5V -> 5V
            • 5V -> ช่องไฟเลี้ยงมอเตอร์
            • GND -> GND
            • 2 -> IN4
            • 3 -> IN3
            • 4 -> IN2
            • 5 -> IN1
L298N Motor Drive Module -> DC Motor มอเตอร์ แนวตั้ง 3-6 Volt
            • OUT1 -> สายสีแดงฝั่งซ้าย
            • OUT2 -> สายสีดำฝั่งซ้าย
            • OUT3 -> สายสีดำฝั่งขวา
            • OUT4 -> สายสีแดงฝั่งขวา
DC Motor มอเตอร์ แนวตั้ง 3-6 Volt -> ใบพัดมอเตอร์
