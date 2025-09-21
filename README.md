# Josjisbot

## Penjelasan singkat
Josjisbot adalah robot holonomic dengan konfigurasi x-drive dan roda omni. Saat ini josjisbot baru berbentuk virtual dan baru bisa dikendalikan menggunakan `teleop_twist_keyboard`, meski demikian josjisbot memiliki dua tipe controller: robot-centric dan world-centric.

![](./x_drive.svg)
### Robot-centric vs world-centric?
Dalam kasus ini robot-centric berarti perintah dari pilot dijalankan oleh robot apa adanya, **tanpa mempertimbangkan arah hadapannya sendiri**. Hal ini dapat membuat pilot yang belum terbiasa kesulitan untuk mengendalikan robot.

![](./robot_centric.gif)
Ilustrasi pergerakan robot-centric

Sedangkan world-centric berarti perintah dari pilot dijalankan oleh robot **dengan mempertimbangkan arah hadapannya sendiri**. Hal ini akan memudahkan pilot yang belum terbiasa mengendalikan robot.

![](./world_centric.gif)
Ilustrasi pergerakan world-centric
